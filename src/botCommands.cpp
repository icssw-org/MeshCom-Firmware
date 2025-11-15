#include <math.h>
#include "loop_functions.h"
#include "botCommands.h"
/*
public code by
DA6SRM

2025-11
*/

struct avgSNR_struct
{
    char callsign[CALLSIGN_MAX_LEN];
    float SNR[MAX_STORED_SNR_PER_STATION];
    float RSSI[MAX_STORED_SNR_PER_STATION];
    uint8_t count_SNR_received;
    uint8_t index;
};

avgSNR_struct *avgSNR_array[MAX_STATIONS_FOR_AVG_SNR];
uint8_t avgSNR_firstEmptyElement = 0;

esp_timer_handle_t periodic_timer;
volatile uint64_t Wifi_Watchdog_Periods;
volatile int badPingCount;



bool HandleBotCommands(String &msg_payload, String source_callsign, String dest_callsign)
{
    char replyBuffer[300];
    bool wasBotCommand = false;
    if (msg_payload.indexOf("!getsnr")>=0 && (strcmp(dest_callsign.c_str(), meshcom_settings.node_call) == 0 || strcmp(dest_callsign.c_str(), "9") == 0))
    {
        Serial.printf("\nsending !getsnr response to %s\n", source_callsign.c_str());
        bool requestPerGroup9 = false;
        String requestingCall = "";                         // report Requester if he is requesting for someone else

        String report_for_callsign = source_callsign;
        if(strcmp(dest_callsign.c_str(), "9"))
        {
            requestPerGroup9 = true;
        }
        if (msg_payload.indexOf("!getsnr=")>=0)             // requesting SNR for specific call instead of source of message
        {
            requestPerGroup9 = true;                        // definitely post this to group 9 so it can't be hidden, since someone is requesting data of another callsign

            int index = msg_payload.indexOf('=');
            int indexEnd = msg_payload.indexOf('{');
            if (msg_payload.length() > index+4)             // callsign must be atleast 3 chars long
            {
                if (indexEnd < index ) indexEnd = msg_payload.length();
                report_for_callsign = msg_payload.substring(index+1, indexEnd);
                report_for_callsign.toUpperCase();
                requestingCall = source_callsign;
            }
            snprintf(replyBuffer, sizeof(replyBuffer), "We sent a reply to %s for !getsnr from %s", source_callsign.c_str(), report_for_callsign.c_str());
            
        }
        else
        {
            snprintf(replyBuffer, sizeof(replyBuffer), "We sent a reply to %s for !getsnr", source_callsign.c_str());

        }
        SendSNRMessage(report_for_callsign, requestPerGroup9, requestingCall);

        //msg_payload = replyBuffer; // replace the payload to reflect that we sent a response
        wasBotCommand = true;
    }
    else if (msg_payload.indexOf("!heard")>=0 && (strcmp(dest_callsign.c_str(), meshcom_settings.node_call) == 0 ))
    {
        SendHeard(source_callsign);
        snprintf(replyBuffer, sizeof(replyBuffer), "We sent a reply to %s for !heard", source_callsign.c_str());

        //msg_payload = replyBuffer; // replace the payload to reflect that we sent a response
        wasBotCommand = true;
    }
    else if (msg_payload.indexOf("!repeat=")>=0 && (strcmp(dest_callsign.c_str(), meshcom_settings.node_call) == 0 ))
    {
        String forwardMessageBuffer;
        int index = msg_payload.indexOf('=');
        int indexEnd = msg_payload.indexOf('{');
        if (msg_payload.length() > index+2)     // don't retransmit very short messages
        {
            if (indexEnd <= 0) indexEnd = msg_payload.length()-1;
            forwardMessageBuffer = msg_payload.substring(index+1, indexEnd);
        }
        SendRepeat(forwardMessageBuffer);

        //snprintf(replyBuffer, sizeof(replyBuffer), "We sent a reply to %s for !repeat=%s", source_callsign.c_str(), forwardMessageBuffer.c_str());

        //msg_payload = replyBuffer; // replace the payload to reflect that we sent a response
        wasBotCommand = true;
    }
    else if (msg_payload.indexOf("!ping")>=0 && (strcmp(dest_callsign.c_str(), meshcom_settings.node_call) == 0 || strcmp(dest_callsign.c_str(), "9") == 0 ))
    {
        SendPingAnswer();
    }
    else if (msg_payload.indexOf("!rebootnow")>=0 && (strcmp(dest_callsign.c_str(), meshcom_settings.node_call) == 0 ))
    {
        ESP.restart();
    }
    return wasBotCommand;
}

void SendSNRMessage(String dest_call, bool requestPerGroup, String requestingCall)
{
    char messageToSend[160];

    String Destination;

    if (!requestPerGroup) Destination = dest_call;
    else Destination = "9";

    if (collectedSamples(dest_call.c_str())<1)
    {
        // didnt receive any packets, sry :3
        snprintf(messageToSend, sizeof(messageToSend), "{%s}I received no packets from %s", Destination.c_str(), dest_call.c_str());

        // no reply if response should go into a group, to reduce spam:
        if (requestPerGroup) return;
    }
    else
    {
        if(requestPerGroup && requestingCall.length()>2)
            snprintf(messageToSend, sizeof(messageToSend), "{9}%s requested the link quality towards %s: SNR=%.1f RSSI=%.1f ", requestingCall.c_str(), dest_call.c_str(), getAverageSNR(dest_call), getAverageRSSI(dest_call));
        else
            snprintf(messageToSend, sizeof(messageToSend), "{%s}Average over %i packtes from %s: SNR=%.1f RSSI=%.1f", Destination.c_str(), collectedSamples(dest_call), dest_call.c_str(), getAverageSNR(dest_call), getAverageRSSI(dest_call));
    }

    sendMessage(messageToSend, strlen(messageToSend));
}

void SendHeard(String dest_call)
{
    char messageToSend[300];

    if (avgSNR_firstEmptyElement == 0)
    {
        snprintf(messageToSend, sizeof(messageToSend), "{%s}I heard no nodes yet", dest_call.c_str());
        // didnt receive any packets, sry :3
    }
    else
    {
        snprintf(messageToSend, sizeof(messageToSend), "{%s}i heard these %i nodes:", dest_call.c_str(), avgSNR_firstEmptyElement);
        for (uint8_t i = 0; i < avgSNR_firstEmptyElement; i++)
        {
            strncat(messageToSend, avgSNR_array[i]->callsign, sizeof(messageToSend)-1);
            strncat(messageToSend, ";", sizeof(messageToSend)-1);
        }
    }
    sendMessage(messageToSend, strlen(messageToSend));
}

void SendRepeat(String message)
{
    char messageToSend[300];

    snprintf(messageToSend, sizeof(messageToSend), "{9}%s", message.c_str());

    sendMessage(messageToSend, strnlen(messageToSend, sizeof(messageToSend)));
}

void SendPingAnswer()
{
    char messageToSend[300];
    int uptime_days = Wifi_Watchdog_Periods/8640;
    int uptime_hours = (Wifi_Watchdog_Periods%8640)/360;
    int uptime_minutes = ((Wifi_Watchdog_Periods%8640)%360)/6;

    snprintf(messageToSend, sizeof(messageToSend), "{9}Pong! Uptime is %iD %iH %iM.", uptime_days, uptime_hours, uptime_minutes);

    sendMessage(messageToSend, strnlen(messageToSend, sizeof(messageToSend)));
}

void AvgSNR_Update(String callsign, float snr, float rssi)
{
    if (callsign.length() > 0 && snr > -30 && rssi > -150)
    {
        bool newCallsign = true;
        uint8_t targetEntry = 0;
        if (avgSNR_firstEmptyElement>0)
        {
            for (int8_t i = 0; i < avgSNR_firstEmptyElement; i++)
            {
                if (avgSNR_array[i] != nullptr)
                {
                //    if (avgSNR_array[i]->callsign.compareTo(callsign) == 0)      // callsign heard before?
                    if (strcmp(avgSNR_array[i]->callsign, callsign.c_str()) == 0)      // callsign heard before?
                    {
                        newCallsign = false;                                    // yes!
                        targetEntry = i;
                        break;
                    }
                }
                else
                    return;
            }
        }
        if (newCallsign)                                                    // init the element, if space for one more
        {
            if (avgSNR_firstEmptyElement == MAX_STATIONS_FOR_AVG_SNR)       // array full, sorry :3
            {
                return;
            }
            
            targetEntry = avgSNR_firstEmptyElement;
            avgSNR_array[avgSNR_firstEmptyElement] = (avgSNR_struct *)malloc(sizeof(avgSNR_struct));
            if (avgSNR_array[avgSNR_firstEmptyElement] == nullptr ) return;     // failed to allocate more memory
            strncpy(avgSNR_array[avgSNR_firstEmptyElement]->callsign, callsign.c_str(), CALLSIGN_MAX_LEN-1);
            avgSNR_array[avgSNR_firstEmptyElement]->count_SNR_received = 0;
            avgSNR_array[avgSNR_firstEmptyElement]->index = 0;
            avgSNR_firstEmptyElement++;
            Serial.printf("=SR= Added new call to AvgSNR-array:%s\n-total calls heard:%i\n", callsign.c_str(), avgSNR_firstEmptyElement);
        }
        avgSNR_array[targetEntry]->SNR[avgSNR_array[targetEntry]->index] = snr;
        avgSNR_array[targetEntry]->RSSI[avgSNR_array[targetEntry]->index] = rssi;

        if (avgSNR_array[targetEntry]->count_SNR_received < MAX_STORED_SNR_PER_STATION)      // how many valid SNR entries?
        {
            avgSNR_array[targetEntry]->count_SNR_received++;
        }
        if (avgSNR_array[targetEntry]->index < MAX_STORED_SNR_PER_STATION-1)                 // index into array
            avgSNR_array[targetEntry]->index++;
        else
            avgSNR_array[targetEntry]->index = 0;
    }
}

float getAverageSNR(String callsign)
{
//    Serial.printf("\n=SR= Averages from <%s>:", callsign.c_str());

    if (callsign.length() > 0)
    {
        bool newCallsign = true;
        uint8_t targetEntry = 0;
        if (avgSNR_firstEmptyElement>0)
        {
            for (int8_t i = 0; i < avgSNR_firstEmptyElement; i++)
            {
                if (strcmp(avgSNR_array[i]->callsign, callsign.c_str()) == 0)      // callsign heard before?
                {
                    newCallsign = false;                                    // yes!
                    targetEntry = i;
                    break;
                }
            }
        }
        if(!newCallsign )
        {  
            float average_snr = 0.0f;

            for (uint8_t i = 0; i < avgSNR_array[targetEntry]->count_SNR_received; i++)          // calculate the average
            {
                average_snr += avgSNR_array[targetEntry]->SNR[i];
            }
            average_snr /= (float)avgSNR_array[targetEntry]->count_SNR_received;
            average_snr = round(average_snr*10)/10;                                             // reduce to 1 decimal place
 //           Serial.printf("  SNR=%.1f\n", average_snr);
            return average_snr;                                                       
        }
    }
    return 0.0f;
}

float getAverageRSSI(String callsign)
{
//    Serial.printf("\n=SR= Averages from <%s>:", callsign.c_str());

    if (callsign.length() > 0)
    {
        bool newCallsign = true;
        uint8_t targetEntry = 0;
        if (avgSNR_firstEmptyElement>0)
        {
            for (int8_t i = 0; i < avgSNR_firstEmptyElement; i++)
            {
                if (strcmp(avgSNR_array[i]->callsign, callsign.c_str()) == 0)      // callsign heard before?
                {
                    newCallsign = false;                                    // yes!
                    targetEntry = i;
                    break;
                }
            }
        }
        if(!newCallsign )
        {  
            float average_rssi = 0.0f;

            for (uint8_t i = 0; i < avgSNR_array[targetEntry]->count_SNR_received; i++)          // calculate the average
            {
                average_rssi += avgSNR_array[targetEntry]->RSSI[i];
            }
            average_rssi /= (float)avgSNR_array[targetEntry]->count_SNR_received;
            average_rssi = round(average_rssi*10)/10;                                             // reduce to 1 decimal place
//            Serial.printf("  RSSI=%.1f  count=%i\n", average_rssi, avgSNR_array[targetEntry]->count_SNR_received);
            return average_rssi;
        }
    }
    return 0.0f;
}

uint8_t collectedSamples(String callsign)
{
    if (callsign.length() > 0)
    {
        bool newCallsign = true;
        uint8_t targetEntry = 0;
        if (avgSNR_firstEmptyElement>0)
        {
            for (int8_t i = 0; i < avgSNR_firstEmptyElement; i++)
            {
                if (strcmp(avgSNR_array[i]->callsign, callsign.c_str()) == 0)      // callsign heard before?
                {
                    newCallsign = false;                                    // yes!
                    targetEntry = i;
                    break;
                }
            }
        }
        if(!newCallsign )
        { 
            return avgSNR_array[targetEntry]->count_SNR_received;
        }
    }
    return 0;
}

void WifiWatchdogSetup()
{
    Serial.printf("=SR= Trying to setup WiFi Watchdog\n", Wifi_Watchdog_Periods, badPingCount);
    uint64_t WifiTimerPeriod = 10000000; // 10 sec interval

    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &WifiWatchdogTimer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "Wifi-Watchdog"
    };

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */

    /* Start the timers */
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, WifiTimerPeriod));
    Wifi_Watchdog_Periods = 0;
    badPingCount = 0;
    Serial.printf("=SR= WiFi Watchdog running\n", Wifi_Watchdog_Periods, badPingCount);
}

static void WifiWatchdogTimer_callback(void* arg)
{
    Wifi_Watchdog_Periods++;
    Serial.printf("=SR= Timer executed n=%i  Bad Ping Count=%i\n", Wifi_Watchdog_Periods, badPingCount);

    if (Wifi_Watchdog_Periods < 360)  // up for less than 1 hour
    {
   //     badPingCount = 0;
    }
    else
    {
        if (badPingCount > 5 || Wifi_Watchdog_Periods > 8640) // reboot after bad connection or 1 day uptime
        {
      //      ESP.restart();
        }        
    }
}

void ReportBadPing()
{
    badPingCount ++;
}
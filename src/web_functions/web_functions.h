#ifndef _WEB_FUNCTIONS_H_
#define _WEB_FUNCTIONS_H_


/**
 * defien some response types we are going to use
 */
#define RESPONSE_TYPE_TEXT 0
#define RESPONSE_TYPE_JSON 1

// WebServer
void startWebserver();
void loopWebserver();
void stopWebserver();



// Common functions
String hex2ascii(String string);
String work_webpage(bool get_password, int webid);


// Pages
void deliver_scaffold();        // delivers the scaffold to the browser. Includes the info-page as initial content
void pwd_webpage();             // ToDo 


// dynamically loaded page contents
void sub_page_info();           // info page
void sub_page_position();       // position page
void sub_page_wx();             // WX page
void sub_page_setup();          // setup page
void sub_page_messages();       // messages page
void sub_page_rxlog();          // RX Log page
void sub_page_path();           // path page
void sub_page_mheard();         // mheard page
void sub_page_spectrum();       // spectrum scan page
void sub_page_unknown();        // a 404 - page
void sub_content_messages();    // only the formatted messages, not the complete message page


// related functions
void send_http_header(uint16_t http_status_code, uint8_t response_type);        // create and send a HTML Header 
void _create_setup_textinput_element(const char id[], const char labelText[], String inputValue, const char placeHolder[], const char parameterName[], uint8_t maxlength);  // create and send a textinput element used in setup
void _create_setup_switch_element(const char id[], const char labelText[], const char descriptionText[], bool checked);     // create and send a switch element used in setup
void _create_meshcom_subheader(String title);                                   // create the sub-page sub-header ( that header containing the date, time and title)

// WEB REST API
void send_message(String);                  // send a message to the mesh
void call_function(String web_header);      // call a node funtion (e.g. sendpos)
void setparam(String);                      // set a parameter (also used in setup)
void getparam(String);                      // get a parameter (also used in setup)

//ToDo: get messages as json


#endif
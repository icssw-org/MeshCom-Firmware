#include <Arduino.h>
#include "compress_functions.h"

void text_compress(String text)
{
    Serial.println("");
    Serial.println("Start:");

    Serial.println(text);
    Serial.printf("Länge unkomprimierter String:%i\n", text.length());

    String compressedText = compress_encode(text, "#");
    Serial.println(compressedText);
    
    Serial.println(compress_decode(compressedText, "#"));
}

/**
 * Komprimiert einen String auf Basis vom Ersetzen von Substrings durch 
 * ein einzelnes Zeichen
 * @param text Der zu komprimierende String
 * @param compressChar Das Zeichen, dass die Substrings ersetzen soll
 * @return Den komprimierten String
 */
String compress_encode(String text, String compressChar)
{
    Serial.printf( "Länge Originalstring:%i\n", text.length());
    //Maximale Ersparnis wird gesucht
    int maxSavings = 0;
    String bestPattern = "";
    //Patternsuche von 2 Zeichen bis text.length / 2
    for ( int pattersize = 2; pattersize <= text.length() / 2; pattersize++ )
    {
        //Kompletten String durchgehen
        for ( int j = 0; j < (text.length() + 1) - pattersize; j++)
        {
            //Pattern bauen
            String pattern = text.substring( j, j + pattersize);
            //Vorkommen zählen
            int found = getOccurences(text, pattern);
            //Ersparnis: Anzahl Funde * Länge Pattern - Länge des Patterns (weil der ja am Anfang wieder hin kommt)+ 1 wegen dem Doppelpunkt + Anzahl Funde wegen dem Char
            int savings = (found * pattern.length()) - (pattern.length()+1+found);
            //Serial.printf( "Pattern:%s  found:%i Ersparnis:%i Zeichen", pattern, found, savings);
            //Die Ersparnis ist größer / gleich gut als das bisher gefundene
            if ( savings > maxSavings )
            {
                //Das Pattern wird gemerkt
                bestPattern = pattern;
                //Ersparnis wird gemerkt
                maxSavings = savings;
            }
        }
    }
    
    //Serial.println( "BestPattern: "+bestPattern );
    String resultString = bestPattern;
    resultString.concat(":");
    text.replace(bestPattern, compressChar);
    resultString.concat(text);

    Serial.printf("Länge komprimierter String:%i\n", resultString.length());
    return resultString;
}

/**
 * Dekomprimiert einen komprimierten String
 * @param text Der komprimierte String
 * @param compressChar Das Kompressionszeichen
 * @return Den dekomrimierten String
 */
String compress_decode(String text, String compressChar)
{
    //Komprimierten String splitten

    //Pattern extrahieren
    String pattern = text.substring(0, text.indexOf(":"));
    //Eigentlichen String extrahieren
    String result = text.substring(text.indexOf(":")+1);
    //Pattern einsetzen
    result.replace(compressChar, pattern);
    return result;
}

/**
 * Liefert die Anzahl zurück, wie oft ein Teilstring in einem String
 * vorkommt
 *
 * @param text Der Gesamttext
 * @param pattern Der Teilstring
 * @return Die Anzahl, wie oft der Teilstring im String vorkommt.
 */
int getOccurences( String text, String pattern )
{
    int occurences = 0;
    boolean eagerMatching = false;
    if ( 0 != pattern.length() )
    {
        for ( int index = text.indexOf( pattern, 0 ); index != -1; index = text
                .indexOf( pattern, eagerMatching ? index + 1 : index + pattern.length() ) )
        {
            occurences++;
        }
        return occurences;
    }
    return 0;
}
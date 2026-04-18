#include <Arduino.h>
#include <configuration.h>

#include "tft_display_functions.h"
#include "tft_custom_colors.h"

#if defined(HAS_TFT)
#include "TFT_eSPI.h"
    TFT_eSPI    tft     = TFT_eSPI(); 
    TFT_eSprite sprite  = TFT_eSprite(&tft);

    void initTFT()
    {
        uint8_t screenBrightness = 50;    //from 1 to 255 to regulate brightness of screens

        delay(500);

        tft.init();
        tft.begin();
        tft.setRotation(1);

        pinMode(TFT_BL, OUTPUT);
        analogWrite(TFT_BL, screenBrightness);
        tft.setTextFont(0);
        tft.fillScreen(TFT_BLACK);
        sprite.createSprite(160,80);
    }

    #define bigSizeFont     2
    #define smallSizeFont   1
    #define lineSpacing     13
    #define maxLineLength   26

    void drawButton(int xPos, int yPos, int wide, int height, String buttonText, int color) {
        uint16_t baseColor, lightColor, darkColor;
        switch (color) {
            case 0:     // Grey Theme
                baseColor   = greyColor;
                lightColor  = greyColorLight;
                darkColor   = greyColorDark;
                break;
            case 1:     // Green Theme
                baseColor   = greenColor;
                lightColor  = greenColorLight;
                darkColor   = greenColorDark;
                break;
            case 2:     // Red Theme
                baseColor   = redColor;
                lightColor  = redColorLight;
                darkColor   = redColorDark;
                break;
            default:    // Fallback color
                baseColor   = 0x0000;   // Black
                lightColor  = 0xFFFF;   // White
                darkColor   = 0x0000;   // Black
                break;
        }

        sprite.fillRect(xPos, yPos, wide, height, baseColor);           // Dibuja el fondo del botón
        sprite.fillRect(xPos, yPos + height - 2, wide, 2, darkColor);   // Línea inferior
        sprite.fillRect(xPos, yPos, wide, 2, lightColor);               // Línea superior
        sprite.fillRect(xPos, yPos, 2, height, lightColor);             // Línea izquierda
        sprite.fillRect(xPos + wide - 2, yPos, 2, height, darkColor);   // Línea derecha
        
        sprite.setTextSize(2);
        sprite.setTextColor(TFT_WHITE, baseColor);

        // Calcula la posición del texto para que esté centrado
        int textWidth = sprite.textWidth(buttonText);           // Ancho del texto
        int textHeight = 16;                                    // Altura aproximada (depende de `setTextSize`)
        int textX = xPos + (wide - textWidth) / 2;              // Centrado horizontal
        int textY = yPos + (height - textHeight) / 2;           // Centrado vertical

        sprite.drawString(buttonText, textX, textY);
    }

    void draw_T_DECK_MenuButtons(int menu) {
        int ladoCuadrado            = 45;
        int curvaCuadrado           = 8;
        int espacioEntreCuadrados   = 18;
        int margenLineaCuadrados    = 10;
        int alturaPrimeraLinea      = 75;
        int alturaSegundaLinea      = 145;
        int16_t colorCuadrados      = 0x2925;
        int16_t colorDestacado      = greyColor;

        for (int i = 0; i < 5; i++) {
            if (i == menu - 1) {
                sprite.fillRoundRect(
                    margenLineaCuadrados + (i * (ladoCuadrado + espacioEntreCuadrados)) - 1,
                    alturaPrimeraLinea - 1,
                    ladoCuadrado + 2,
                    ladoCuadrado + 2,
                    curvaCuadrado,
                    TFT_WHITE
                );
                sprite.fillRoundRect(
                    margenLineaCuadrados + (i * (ladoCuadrado + espacioEntreCuadrados)),
                    alturaPrimeraLinea,
                    ladoCuadrado,
                    ladoCuadrado,
                    curvaCuadrado,
                    TFT_BLACK
                );
                sprite.fillRoundRect(
                    margenLineaCuadrados + (i * (ladoCuadrado + espacioEntreCuadrados)),    // x-coordinate
                    alturaPrimeraLinea,                                                     // y-coordinate
                    ladoCuadrado,                                                           // width
                    ladoCuadrado,                                                           // height
                    curvaCuadrado,                                                          // corner radius
                    colorDestacado                                                          // color
                );
            } else {
                sprite.fillRoundRect(
                    margenLineaCuadrados + (i * (ladoCuadrado + espacioEntreCuadrados)),    // x-coordinate
                    alturaPrimeraLinea,                                                     // y-coordinate
                    ladoCuadrado,                                                           // width
                    ladoCuadrado,                                                           // height
                    curvaCuadrado,                                                          // corner radius
                    colorCuadrados                                                          // color
                );
            }
            sprite.fillRoundRect(
                margenLineaCuadrados + (i * (ladoCuadrado + espacioEntreCuadrados)),    // x-coordinate
                alturaSegundaLinea,                                                     // y-coordinate
                ladoCuadrado,                                                           // width
                ladoCuadrado,                                                           // height
                curvaCuadrado,                                                          // corner radius
                colorCuadrados                                                          // color
            );
        }
    }

    void displayTFT(const String& header)
    {
        sprite.fillRect(0, 0, 160, 19, redColor);
        sprite.setTextFont(2);
        sprite.setTextSize(smallSizeFont);
        sprite.setTextColor(TFT_WHITE, redColor);
        sprite.drawString(header, 3, 2);

        sprite.pushSprite(0,0);
    }

    void displayTFT(const String& header, const String& line)
    {
        sprite.fillSprite(TFT_BLACK); 
        sprite.fillRect(0, 0, 160, 19, redColor);
        sprite.setTextFont(2);
        sprite.setTextSize(smallSizeFont);
        sprite.setTextColor(TFT_WHITE, redColor);
        sprite.drawString(header, 3, 2);

        sprite.setTextSize(smallSizeFont);
        sprite.setTextColor(TFT_WHITE, TFT_BLACK);

        int lspace = lineSpacing;
        int yLineOffset = (lspace * 2) - 8;

        sprite.setCursor(2, yLineOffset);

        sprite.setTextWrap(true, false);

        sprite.println(line);

        sprite.pushSprite(0,0);
    }

    void displayTFT(const String& header, const String& line1, const String& line2, const String& line3, const String& line4, int wait)
    {
        sprite.fillSprite(TFT_BLACK); 
        sprite.fillRect(0, 0, 160, 19, redColor);
        sprite.setTextFont(0);
        sprite.setTextSize(bigSizeFont);
        sprite.setTextColor(TFT_WHITE, redColor);
        sprite.drawString(header, 3, 2);

        const String* const lines[] = {&line1, &line2, &line3, &line4};

        sprite.setTextFont(2);
        sprite.setTextSize(smallSizeFont);
        sprite.setTextColor(TFT_WHITE, TFT_BLACK);

        int lspace = lineSpacing;
        int yLineOffset = (lspace * 2) - 8;

        for (int i = 0; i < 4; i++) {
            String text = *lines[i];
            if (text.length() > 0)
            {                    
                while (text.length() > 0)
                {
                    String chunk = text.substring(0, maxLineLength);
                    sprite.drawString(chunk, 3, yLineOffset);
                    text = text.substring(maxLineLength);
                    yLineOffset += lspace;
                }
            }
            else
            {
                sprite.drawString(text, 3, yLineOffset);
                yLineOffset += lspace;
            }
        }
        sprite.pushSprite(0,0);

        delay(wait);
    }

    void displayTFT(const String& header, const String& line1, const String& line2, const String& line3, const String& line4, const String& line5, int wait)
    {
        sprite.fillSprite(TFT_BLACK); 
        sprite.fillRect(0, 0, 160, 19, redColor);
        sprite.setTextFont(2);
        sprite.setTextSize(smallSizeFont);
        sprite.setTextColor(TFT_WHITE, redColor);
        sprite.drawString(header, 3, 2);

        const String* const lines[] = {&line1, &line2, &line3, &line4, &line5};

        sprite.setTextSize(smallSizeFont);
        sprite.setTextColor(TFT_WHITE, TFT_BLACK);

        int lspace = lineSpacing;
        int yLineOffset = (lspace * 2) - 8;

        for (int i = 0; i < 5; i++) {
            String text = *lines[i];
            if (text.length() > 0) {
                while (text.length() > 0) {
                    String chunk = text.substring(0, maxLineLength);
                    sprite.drawString(chunk, 3, yLineOffset);
                    text = text.substring(maxLineLength);
                    yLineOffset += lspace;
                }
            } else {
                sprite.drawString(text, 3, yLineOffset);
                yLineOffset += lspace;
            }
        }
        sprite.pushSprite(0,0);
    
        delay(wait);
    }
#endif

#if defined(HAS_TFT_114)

// T114 TFT_WIDTH = 135 TFT_HEIGHT = 240 TFT_RORATION = 3 (so WIDTH and HEIGHT changed in use)
// 
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

    extern Adafruit_ST7789 tft1;

    void displayTFT(const String& header)
    {
        //Serial.println("[DISP]...1 Line");

        tft1.fillRect(0, 0, TFT_HEIGHT, 30, ST77XX_RED);
        tft1.setCursor(4, 6);
        tft1.setTextColor(ST77XX_WHITE);
        tft1.setTextSize(2); //2... 12*16
        tft1.println(header);
    }

    void displayTFT(const String& header, const String& line)
    {
        //Serial.println("[DISP]...2 Lines");

        tft1.fillScreen(ST77XX_WHITE);

        tft1.fillRect(0, 0, TFT_HEIGHT, 30, ST77XX_RED);
        tft1.setCursor(4, 6);
        tft1.setTextColor(ST77XX_WHITE);
        tft1.setTextSize(2); //2... 12*16
        tft1.println(header);

        tft1.setTextColor(ST77XX_BLACK);
        tft1.setTextWrap(true);
        tft1.setTextSize(2); //2... 12*16
        tft1.setCursor(0, 32);
        tft1.println(line);

    }

    void displayTFT(const String& header, const String& line1, const String& line2, const String& line3, const String& line4, int wait)
    {
        //Serial.println("[DISP]...5 Lines");

        tft1.fillScreen(ST77XX_WHITE);

        tft1.fillRect(0, 0, TFT_HEIGHT, 30, ST77XX_RED);
        tft1.setCursor(0, 0);
        tft1.setTextColor(ST77XX_WHITE);
        tft1.setTextSize(3); //2... 12*16
        tft1.println(header);

        tft1.setTextColor(ST77XX_BLACK);
        tft1.setTextSize(2); //2... 12*16
        tft1.println("");
        tft1.println(line1);
        tft1.println("");

        tft1.setTextSize(3); //2... 12*16
        tft1.println(line2);
        tft1.setTextSize(2); //2... 12*16
        tft1.println(line3);
        tft1.println(line4);
        
        //tft.setTextWrap(false);
    }

    void displayTFT(const String& header, const String& line1, const String& line2, const String& line3, const String& line4, const String& line5, int wait)
    {
        //Serial.println("[DISP]...6 Lines");

        tft1.fillScreen(ST77XX_WHITE);

        tft1.fillRect(0, 0, TFT_HEIGHT, 30, ST77XX_RED);
        tft1.setCursor(4, 6);
        tft1.setTextColor(ST77XX_WHITE);
        tft1.setTextSize(2); //2... 12*16
        tft1.println(header);

        tft1.setTextColor(ST77XX_BLACK);
        tft1.setCursor(0, 32);
        tft1.println(line1);
        tft1.println(line2);
        tft1.println(line3);
        tft1.println(line4);
        tft1.println(line5);
    }

#endif
/*
 * Lab 4: PONG
 * Created: Nov 13, 2023
 * Author : vbwanere
 */

#include <avr/io.h> 
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "uart.h"    
// #include "ST7735.h"  // LCD library.
// #include "LCD_GFX.h" // LCD graphics library.
// #include "setup.h"   // contains the configureProject() function.


#define F_CPU 16000000UL // 16 MHz clock speed.
#include <util/delay.h>
#define BAUD_RATE 9600
#define BAUD_PRESCALER ((F_CPU / (BAUD_RATE * 16UL)) - 1)
/*#################### Include necessary libraries above #####################*/

/*******************************************************************/
//                              PONG Game                          //
/*******************************************************************/
/*
char String[25];  // define character array.
uint8_t human_paddle_start = 5; // starting point of the human paddle.
uint8_t human_paddle_ends = 30; // ending point of the human paddle.

uint8_t computer_paddle_starts = 5; // starting point of the computer paddle.
uint8_t computer_paddle_ends = 30; // ending point of the computer paddle.

// flag to indicate if the computer paddle and ball have reached the extremes:
uint8_t computer_reached = 0; 
uint8_t ball_reached = 0;


int main(void) {
    configureProject(); // configure MCU, ADC, LED, TFT.
    UART_init(BAUD_PRESCALER); 
    LCD_setScreen(WHITE);

    int human_score = 0; // initialize human score.
    char human_score_char[1]; // character array to store human score.
    int game_round = 0; // initialize game round.
    char game_round_char[1]; // character array to store game round.
    int computer_score = 0; // initialize computer score. 
    char computer_score_char[1]; // character array to store computer score.

    int ball_x0 = 80;
    int ball_y0 = 80;
    int ball_x1 = 82;
    int ball_y1 = 82;
    int w = abs(ball_x1 - ball_x0);
    int sw = ball_x0 < ball_x1 ? 1:-1;
    int h = -abs(ball_y1 - ball_y0);
    int sh = ball_y0 < ball_y1 ? 1:-1;
    int error = w + h;

    while (1) {
        // Displaying the score and round:
        sprintf(human_score_char, "%d", human_score);
        sprintf(computer_score_char, "%d", computer_score);
        sprintf(game_round_char, "%d", game_round);
        LCD_drawChar(10, 5, human_score_char[0], BLUE, WHITE);
        LCD_drawChar(140, 5, computer_score_char[0], BLUE, WHITE);
        LCD_drawChar(80, 5, game_round_char[0], RED, YELLOW);

        if(computer_reached == 0){
            LCD_drawBlock(155, computer_paddle_starts, 159, computer_paddle_ends, BLUE);
            LCD_drawBlock(155, computer_paddle_starts, 159, computer_paddle_ends, WHITE);
            computer_paddle_starts += 1;
            computer_paddle_ends += 1;
            if(computer_paddle_ends >= 127){
                computer_reached = 1;
            }
        }

        if(computer_reached == 1){
            LCD_drawBlock(155, computer_paddle_starts, 159, computer_paddle_ends, BLUE);
            LCD_drawBlock(155, computer_paddle_starts, 159, computer_paddle_ends, WHITE);
            computer_paddle_starts -= 1;
            computer_paddle_ends -= 1;
            if(computer_paddle_ends <= 24){
                computer_reached = 0;
            }
        }
        
        // joystick paddle:
        if((ADC <= 600 && ADC >= 400)) {
            LCD_drawBlock(0, human_paddle_start, 3, human_paddle_ends, BLUE);
        }

        if(ADC < 400 || PINC & (1<<PINC3)) {
            if(human_paddle_ends > 24){
                LCD_drawBlock(0, human_paddle_start, 3, human_paddle_ends, BLUE);
                LCD_drawBlock(0, human_paddle_start, 3, human_paddle_ends, WHITE);
                human_paddle_start -= 1;
                human_paddle_ends -= 1;
            }
            if(ADC > 600 || PINC & (1<<PINC4)){
                continue;
            }
            if(human_paddle_ends <= 24){
                human_paddle_start = 0;
                human_paddle_ends = 24;
                LCD_drawBlock(0, human_paddle_start, 3, human_paddle_ends, BLUE);
                LCD_drawBlock(0, human_paddle_start, 3, human_paddle_ends, WHITE);
            }
        }
        if(ADC > 600 || PINC & (1<<PINC4)){
            if(human_paddle_start < 103) {
                LCD_drawBlock(0, human_paddle_start, 3, human_paddle_ends, BLUE);
                LCD_drawBlock(0, human_paddle_start, 3, human_paddle_ends, WHITE);
                human_paddle_start += 1;
                human_paddle_ends += 1;
            }
            if(ADC < 400 || PINC & (1<<PINC3)){
                continue;
            }
            if(human_paddle_start >= 103){
                human_paddle_start = 103;
                human_paddle_ends = 127;
                LCD_drawBlock(0, human_paddle_start, 3, human_paddle_ends, BLUE);
                LCD_drawBlock(0, human_paddle_start, 3, human_paddle_ends, WHITE);
            }
        }
        
        // bouncing ball:
        // LCD_drawCircle(ball_x0, ball_y0, 5, RED); // does not work.
        // LCD_drawCircle(ball_x0, ball_y0, 5, WHITE); // does not work.

        if (ball_x0 <= 0 || ball_x0 >= 159){
            sw = -sw;
        }
        if(ball_y0 <= 0 || ball_y0 >= 127){
            sh = -sh;
        }
        int error2= 2 * error;
        if (error2 >= h){
            error += h;
            ball_x0 +=sw;
        }
        if (error2 <= w){
            error += w;
            ball_y0 += sh;
        }

        // computer wins
        if(ball_x0 <= 0 && (ball_y0>human_paddle_ends || ball_y0<human_paddle_start)) {
            PORTC |= (1<<PORTC1);
            for(int i = 0; i < 10; i++){
                PORTD |= (1<<PORTD3);
                _delay_ms(65);
                PORTD &= ~ (1<<PORTD3);
                _delay_ms(35);
            }
            PORTC &= ~ (1<<PORTC1);
            if(computer_score <= 1){
                computer_score += 1;
            }
            else {
                computer_score = 0;
                human_score = 0;
                if(game_round <= 1){
                    game_round += 1;
                }
                else{
                    game_round = 0;
                }
            }

            sprintf(String, "computer human_score %d \r\n", computer_score);
            UART_putstring(String);

            ball_x0 = 80;
            ball_y0 = (rand() % (94 - 34 + 1)) + 34;
            human_paddle_start = 52;
            human_paddle_ends = 76;
            computer_paddle_starts = 0;
            computer_paddle_ends = 24;
            computer_reached = 0;
            if(ball_reached == 0){
                ball_x1 = 78;
                ball_y1 = ball_y0 - 2;
                ball_reached = 1;
            }
            else {
                ball_x1 = 82;
                ball_y1 = ball_y0 + 2;
                ball_reached = 0;
            }
            w = abs(ball_x1-ball_x0);
            sw = ball_x0<ball_x1 ? 1:-1;
            h = -abs(ball_y1-ball_y0);
            sh = ball_y0<ball_y1 ? 1:-1;
            error = w+h;
            LCD_setScreen(WHITE);
        }


        //human wins
        if(ball_x0 >= 159 && (ball_y0>computer_paddle_ends || ball_y0<computer_paddle_starts)){
            PORTC |= (1<<PORTC2);
            for(int i = 0; i < 10; i++){
                PORTD |= (1<<PORTD3);
                _delay_ms(65);
                PORTD &= ~ (1<<PORTD3);
                _delay_ms(35);
            }
            PORTC &= ~ (1<<PORTC2);
            if(human_score <= 1) {
                human_score += 1;
            }
            else {
                human_score = 0;
                computer_score = 0;
                if(game_round <= 1){
                    game_round += 1;
                }
                else{
                    game_round = 0;
                }
            }

            sprintf(String, "human human_score %d \r\n", human_score);
            UART_putstring(String);
            ball_x0 = 80;
            ball_y0 = (rand() % (94 - 34 + 1)) + 34;
            human_paddle_start = 52;
            human_paddle_ends = 76;
            computer_paddle_starts = 0;
            computer_paddle_ends = 24;
            computer_reached = 0;
            if(ball_reached == 0){
                ball_x1 = 78;
                ball_y1 = ball_y0 - 2;
                ball_reached = 1;
                }
            else {
                ball_x1 = 82;
                ball_y1 = ball_y0 + 2;
                ball_reached = 0;
            }
            w = abs(ball_x1-ball_x0);
            sw = ball_x0<ball_x1 ? 1:-1;
            h = -abs(ball_y1-ball_y0);
            sh = ball_y0<ball_y1 ? 1:-1;
            error = w+h;
            LCD_setScreen(WHITE);
        }
    }
}
*/
/****************************************************************************************/
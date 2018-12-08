//header for morse code stuff
#ifndef MORSE_H
#define MORSE_H


struct node {
	unsigned  char letter;
	struct node* left;
	struct node* right;
};
typedef  struct node   letter;


void wait (int ticks);
void morseOut(char letter);
void buzzz(int a, int b);
char morseIn();
void space();

//morse code conversion code inspired by pervious year's project
//https://spaces.usu.edu/display/ece3710/Morse+Code+Translator+2?preview=/90290816/91357239/Morse%20Code%20Translator.pdf



#endif

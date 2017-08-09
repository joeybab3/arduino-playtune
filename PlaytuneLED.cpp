#include <Arduino.h>
#include "PlaytuneLED.h"

#ifndef DBUG
#define DBUG 0          // debugging?
#endif
#define ASSUME_VOLUME 0 // assume volume information is present in bytestream files without headers?
#define TESLA_COIL 0    // special Tesla Coil version?


struct file_hdr_t {  // the optional bytestream file header
  char id1;     // 'P'
  char id2;     // 't'
  unsigned char hdr_length; // length of whole file header
  unsigned char f1;         // flag byte 1
  unsigned char f2;         // flag byte 2
  unsigned char num_tgens;  // how many tone generators are used by this score
} file_header;
#define HDR_F1_VOLUME_PRESENT 0x80
#define HDR_F1_INSTRUMENTS_PRESENT 0x40
#define HDR_F1_PERCUSSION_PRESENT 0x20

// timer ports and masks

#if defined(__AVR_ATmega8__)
#define TCCR2A TCCR2
#define TCCR2B TCCR2
#define COM2A1 COM21
#define COM2A0 COM20
#define OCR2A OCR2
#define TIMSK2 TIMSK
#define OCIE2A OCIE2
#define TIMER2_COMPA_vect TIMER2_COMP_vect
#define TIMSK1 TIMSK
#endif

#if !defined(__AVR_ATmega8__)
volatile byte *timer0_pin_port;
volatile byte timer0_pin_mask;
#endif
volatile byte *timer1_pin_port;
volatile byte timer1_pin_mask;
#if !defined(__AVR_ATmega32U4__)
volatile byte *timer2_pin_port;
volatile byte timer2_pin_mask;
#endif
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)||defined(__AVR_ATmega32U4__)
volatile byte *timer3_pin_port;
volatile byte timer3_pin_mask;
volatile byte *timer4_pin_port;
volatile byte timer4_pin_mask;
#endif
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)
volatile byte *timer5_pin_port;
volatile byte timer5_pin_mask;
#endif

// Define the order to allocate timers.

#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)
#define AVAILABLE_TIMERS 6
const byte PROGMEM tune_pin_to_timer_PGM[] = {
  1, 2, 3, 4, 5, 0
};
#elif defined(__AVR_ATmega8__)
#define AVAILABLE_TIMERS 2
const byte PROGMEM tune_pin_to_timer_PGM[] = {
  1, 2
};
#elif defined(__AVR_ATmega32U4__)
#define AVAILABLE_TIMERS 4
const byte PROGMEM tune_pin_to_timer_PGM[] = {
  1, 0, 3, 4
};
#else
#define AVAILABLE_TIMERS 3
const byte PROGMEM tune_pin_to_timer_PGM[] = {
  1, 2, 0
};
#endif

//  Other local varables

byte _tune_pins[AVAILABLE_TIMERS];
byte _tune_num_chans = 0;

/* one of the timers is also used to time
  - score waits (whether or not that timer is playing a note)
  - tune_delay() delay requests
  We currently use timer1, since that is the common one available on different microcontrollers.
*/
volatile unsigned wait_timer_frequency2;       /* its current frequency */
volatile unsigned wait_timer_old_frequency2;   /* its previous frequency */
volatile boolean wait_timer_playing = false;   /* is it currently playing a note? */
volatile boolean doing_delay = false;          /* are we using it for a tune_delay()? */
volatile unsigned long wait_toggle_count;      /* countdown score waits */
volatile unsigned long delay_toggle_count;     /* countdown tune_ delay() delays */

volatile const byte *score_start = 0;
volatile const byte *score_cursor = 0;
volatile boolean PlaytuneLED::tune_playing = false;
boolean volume_present = ASSUME_VOLUME;

// Table of midi note frequencies * 2
//   They are times 2 for greater accuracy, yet still fit in a word.
//   Generated from Excel by =ROUND(2*440/32*(2^((x-9)/12)),0) for 0<x<128
// The lowest notes might not work, depending on the Arduino clock frequency

const unsigned int PROGMEM tune_frequencies2_PGM[128] =
{
  16, 17, 18, 19, 21, 22, 23, 24, 26, 28, 29, 31, 33, 35, 37, 39, 41,
  44, 46, 49, 52, 55, 58, 62, 65, 69, 73, 78, 82, 87, 92, 98, 104, 110,
  117, 123, 131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233,
  247, 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494,
  523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1047,
  1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
  2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729,
  3951, 4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040,
  7459, 7902, 8372, 8870, 9397, 9956, 10548, 11175, 11840, 12544,
  13290, 14080, 14917, 15804, 16744, 17740, 18795, 19912, 21096,
  22351, 23680, 25088
};

void tune_playnote (byte chan, byte note);
void tune_stopnote (byte chan);
void tune_stepscore (void);

const uint16_t PixelCount = 127; // this example assumes 4 pixels, making it smaller will cause a failure
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount,5);

RgbColor red(255, 0, 0);
RgbColor green(0, 255, 0);
RgbColor blue(0, 0, 255);
RgbColor yellow(255, 255, 0);
RgbColor pink(0, 255, 255);
RgbColor white(255);
RgbColor black(0);
RgbColor colors[6] = {blue,green,red,yellow,pink,white};
byte lastnote[6] = {};

#if TESLA_COIL
void teslacoil_rising_edge(byte timernum);
byte teslacoil_checknote(byte note);
#endif

//------------------------------------------------------
// Initialize a music channel on a specific output pin
//------------------------------------------------------
PlaytuneLED::PlaytuneLED()	{
  
}

void PlaytuneLED::tune_initchan(byte pin) {
  byte timer_num;

  if (_tune_num_chans < AVAILABLE_TIMERS) {
    timer_num = pgm_read_byte(tune_pin_to_timer_PGM + _tune_num_chans);
    _tune_pins[_tune_num_chans] = pin;
    _tune_num_chans++;
    pinMode(pin, OUTPUT);
#if DBUG
    Serial.print("init pin "); Serial.print(pin);
    Serial.print(" on timer "); Serial.println(timer_num);
#endif
    switch (timer_num) { // All timers are put in CTC mode

#if !defined(__AVR_ATmega8__)
      case 0:  // 8 bit timer
        TCCR0A = 0;
        TCCR0B = 0;
        bitWrite(TCCR0A, WGM01, 1);
        bitWrite(TCCR0B, CS00, 1);
        timer0_pin_port = portOutputRegister(digitalPinToPort(pin));
        timer0_pin_mask = digitalPinToBitMask(pin);
        break;
#endif
      case 1:  // 16 bit timer
        TCCR1A = 0;
        TCCR1B = 0;
        bitWrite(TCCR1B, WGM12, 1);
        bitWrite(TCCR1B, CS10, 1);
        timer1_pin_port = portOutputRegister(digitalPinToPort(pin));
        timer1_pin_mask = digitalPinToBitMask(pin);
        tune_playnote (0, 60);  /* start and stop channel 0 (timer 1) on middle C so wait/delay works */
        tune_stopnote (0);
        break;
#if !defined(__AVR_ATmega32U4__)
      case 2:  // 8 bit timer
        TCCR2A = 0;
        TCCR2B = 0;
        bitWrite(TCCR2A, WGM21, 1);
        bitWrite(TCCR2B, CS20, 1);
        timer2_pin_port = portOutputRegister(digitalPinToPort(pin));
        timer2_pin_mask = digitalPinToBitMask(pin);
        break;
#endif
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)||defined(__AVR_ATmega32U4__)
      case 3:  // 16 bit timer
        TCCR3A = 0;
        TCCR3B = 0;
        bitWrite(TCCR3B, WGM32, 1); // CTC mode
        bitWrite(TCCR3B, CS30, 1);  // clk/1 (no prescaling)
        timer3_pin_port = portOutputRegister(digitalPinToPort(pin));
        timer3_pin_mask = digitalPinToBitMask(pin);
        break;
#endif
#if defined(__AVR_ATmega32U4__)
      case 4: // 10 bit timer, treated as 8 bit
        TCCR4A = 0;
        TCCR4B = 0;
        bitWrite(TCCR4B, CS40, 1); // clk/1 (no prescaling)
        timer4_pin_port = portOutputRegister(digitalPinToPort(pin));
        timer4_pin_mask = digitalPinToBitMask(pin);
        break;
#endif
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)
      case 4:  // 16 bit timer
        TCCR4A = 0;
        TCCR4B = 0;
        bitWrite(TCCR4B, WGM42, 1);
        bitWrite(TCCR4B, CS40, 1);
        timer4_pin_port = portOutputRegister(digitalPinToPort(pin));
        timer4_pin_mask = digitalPinToBitMask(pin);
        break;
      case 5:  // 16 bit timer
        TCCR5A = 0;
        TCCR5B = 0;
        bitWrite(TCCR5B, WGM52, 1);
        bitWrite(TCCR5B, CS50, 1);
        timer5_pin_port = portOutputRegister(digitalPinToPort(pin));
        timer5_pin_mask = digitalPinToBitMask(pin);
        break;
#endif
    }
  }
}

//-----------------------------------------------
// Start playing a note on a particular channel
//-----------------------------------------------

void tune_playnote (byte chan, byte note) {
  byte timer_num;
  byte prescalarbits = 0b001;
  unsigned int frequency2; /* frequency times 2 */
  unsigned long ocr;
  strip.SetPixelColor(lastnote[chan], black);
  lastnote[chan] = note;
  strip.SetPixelColor(note, colors[chan]);
  strip.SetPixelColor(chan, colors[chan]);
  strip.Show();
  Serial.print("lit: "+String(note));
  Serial.println(" color: "+String(chan));
  Serial.print("[");
  Serial.print(String(lastnote[0])+" ");
  Serial.print(String(lastnote[1])+" ");
  Serial.print(String(lastnote[2])+" ");
  Serial.print(String(lastnote[3])+" ");
  Serial.print(String(lastnote[4])+" ");
  Serial.print(String(lastnote[5])+"]");
  Serial.println();
  
#if DBUG
  Serial.print ("Play at ");
  Serial.print(score_cursor - score_start, HEX);
  Serial.print(", ch");
  Serial.print(chan); Serial.print(' ');
  Serial.println(note, HEX);
#endif
  if (chan < _tune_num_chans) {
    timer_num = pgm_read_byte(tune_pin_to_timer_PGM + chan);
#if TESLA_COIL
    note = teslacoil_checknote(note);  // let teslacoil modify the note
#endif
    if (note > 127) note = 127;
    frequency2 = pgm_read_word (tune_frequencies2_PGM + note);
    // The stuff below really needs a rewrite to avoid so many divisions and to
    // make it easier to add new processors with different timer configurations!
    if (timer_num == 0 || timer_num == 2
#if defined(__AVR_ATmega32U4__)
        || timer_num == 4 // treat the 10-bit counter as an 8-bit counter
#endif
       ) { //***** 8 bit timer ******
      if (note < ( F_CPU <= 8000000UL ? 12 : 24))
        return;   //  too low to be playable
      // scan through prescalars to find the best fit
      ocr = F_CPU / frequency2 - 1;
      prescalarbits = 0b001;  // ck/1: same for all timers
      if (ocr > 255) {
        ocr = F_CPU / frequency2 / 8 - 1;
        prescalarbits = timer_num == 4 ? 0b0100 : 0b010;  // ck/8
        if (timer_num == 2 && ocr > 255) {
          ocr = F_CPU / frequency2 / 32 - 1;
          prescalarbits = 0b011; // ck/32
        }
        if (ocr > 255) {
          ocr = F_CPU / frequency2 / 64 - 1;
          prescalarbits = timer_num == 0 ? 0b011 : (timer_num == 4 ? 0b0111 : 0b100);  // ck/64
          if (timer_num == 2 && ocr > 255) {
            ocr = F_CPU / frequency2 / 128 - 1;
            prescalarbits = 0b101; // ck/128
          }
          if (ocr > 255) {
            ocr = F_CPU / frequency2 / 256 - 1;
            prescalarbits = timer_num == 0 ? 0b100 : (timer_num == 4 ? 0b1001 : 0b110); // clk/256
            if (ocr > 255) {
              // can't do any better than /1024
              ocr = F_CPU / frequency2 / 1024 - 1;
              prescalarbits = timer_num == 0 ? 0b101 : (timer_num == 4 ? 0b1011 : 0b111); // clk/1024
            }
          }
        }
      }
#if !defined(__AVR_ATmega8__)
      if (timer_num == 0) TCCR0B = (TCCR0B & 0b11111000) | prescalarbits;
#if defined(__AVR_ATmega32U4__)
      else if (timer_num == 4) {
        TCCR4B = (TCCR4B & 0b11110000) | prescalarbits;
      }
#endif
      else { // must be timer_num == 2
#endif
#if !defined(__AVR_ATmega32U4__)
        TCCR2B = (TCCR2B & 0b11111000) | prescalarbits;
#endif
      }
    }
    else  //******  16-bit timer  *********
    { // two choices for the 16 bit timers: ck/1 or ck/64
      ocr = F_CPU / frequency2 - 1;
      prescalarbits = 0b001;
      if (ocr > 0xffff) {
        ocr = F_CPU / frequency2 / 64 - 1;
        prescalarbits = 0b011;
      }
      if (timer_num == 1) TCCR1B = (TCCR1B & 0b11111000) | prescalarbits;
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)||defined(__AVR_ATmega32U4__)
      else if (timer_num == 3) TCCR3B = (TCCR3B & 0b11111000) | prescalarbits;
#endif
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)
      else if (timer_num == 4) TCCR4B = (TCCR4B & 0b11111000) | prescalarbits;
      else if (timer_num == 5) TCCR5B = (TCCR5B & 0b11111000) | prescalarbits;
#endif
    }

    // Set the OCR for the timer, zero the counter, then turn on the interrupts
    switch (timer_num) {
#if !defined(__AVR_ATmega8__)
      case 0:
        OCR0A = ocr;
        TCNT0 = 0;
        bitWrite(TIMSK0, OCIE0A, 1);
        break;
#endif
      case 1:
        OCR1A = ocr;
        TCNT1 = 0;
        wait_timer_frequency2 = frequency2;  // for "tune_delay" function
        wait_timer_playing = true;
        bitWrite(TIMSK1, OCIE1A, 1);
        break;
#if !defined(__AVR_ATmega32U4__)
      case 2:
        OCR2A = ocr;
        TCNT2 = 0;
        bitWrite(TIMSK2, OCIE2A, 1);
        break;
#endif
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)||(__AVR_ATmega32U4__)
      case 3:
        OCR3A = ocr;
        TCNT3 = 0;
        bitWrite(TIMSK3, OCIE3A, 1);
        break;
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)
      case 4:
        OCR4A = ocr;
        TCNT4 = 0;
        bitWrite(TIMSK4, OCIE4A, 1);
        break;
#endif
#if defined(__AVR_ATmega32U4__)
      case 4:// TOP value compare for this 10-bit register is in C!
        OCR4C = ocr / 2 + 1; //timer4 doesn't have CTC mode, but I don't understand the f/2
        // others have reported problems too, and apparently the chip as has bugs.
        // http://forum.arduino.cc/index.php?topic=261869.0
        // http://electronics.stackexchange.com/questions/245661/atmega32u4-generate-clock-using-timer4
        TCNT4 = 0;
        bitWrite(TIMSK4, OCIE4A, 1);
        break;
#endif
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)
      case 5:
        OCR5A = ocr;
        TCNT5 = 0;
        bitWrite(TIMSK5, OCIE5A, 1);
        break;
#endif
#endif
    }
  }
}

//-----------------------------------------------
// Stop playing a note on a particular channel
//-----------------------------------------------

void tune_stopnote (byte chan) {
  byte timer_num;
  strip.SetPixelColor(lastnote[chan], black);
  strip.Show();
#if DBUG
  Serial.print ("Stop note ");
  Serial.println(chan, DEC);
#endif

  timer_num = pgm_read_byte(tune_pin_to_timer_PGM + chan);
  switch (timer_num) {
#if !defined(__AVR_ATmega8__)
    case 0:
      TIMSK0 &= ~(1 << OCIE0A);                 // disable the interrupt
      *timer0_pin_port &= ~(timer0_pin_mask);   // keep pin low after stop
      break;
#endif
    case 1:
      // We leave the timer1 interrupt running for timing delays and score waits
      wait_timer_playing = false;
      *timer1_pin_port &= ~(timer1_pin_mask);   // keep pin low after stop
      break;
#if !defined(__AVR_ATmega32U4__)
    case 2:
      TIMSK2 &= ~(1 << OCIE1A);                 // disable the interrupt
      *timer2_pin_port &= ~(timer2_pin_mask);   // keep pin low after stop
      break;
#endif
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)||defined(__AVR_ATmega32U4__)
    case 3:
      TIMSK3 &= ~(1 << OCIE3A);                 // disable the interrupt
      *timer3_pin_port &= ~(timer3_pin_mask);   // keep pin low after stop
      break;
    case 4:
      TIMSK4 &= ~(1 << OCIE4A);                 // disable the interrupt
      *timer4_pin_port &= ~(timer4_pin_mask);   // keep pin low after stop
      break;
#endif
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)
    case 5:
      TIMSK5 &= ~(1 << OCIE5A);                 // disable the interrupt
      *timer5_pin_port &= ~(timer5_pin_mask);   // keep pin low after stop
      break;
#endif
  }
}

//-----------------------------------------------
// Start playing a score
//-----------------------------------------------

void PlaytuneLED::tune_playscore (const byte *score) {
  if (tune_playing) tune_stopscore();
  strip.Begin();
  score_start = score;
  volume_present = ASSUME_VOLUME;

  // look for the optional file header
  memcpy_P(&file_header, score, sizeof(file_hdr_t)); // copy possible header from PROGMEM to RAM
  if (file_header.id1 == 'P' && file_header.id2 == 't') { // validate it
    volume_present = file_header.f1 & HDR_F1_VOLUME_PRESENT;
#if DBUG
    Serial.print("header: volume_present="); Serial.println(volume_present);
#endif
    score_start += file_header.hdr_length; // skip the whole header
  }
  score_cursor = score_start;
  tune_stepscore();  /* execute initial commands */
  PlaytuneLED::tune_playing = true;  /* release the interrupt routine */
}

void tune_stepscore (void) {
  byte cmd, opcode, chan, note;
  unsigned duration;
  /* Do score commands until a "wait" is found, or the score is stopped.
    This is called initially from tune_playcore, but then is called
    from the interrupt routine when waits expire.
  */
#define CMD_PLAYNOTE	0x90	/* play a note: low nibble is generator #, note is next byte */
#define CMD_STOPNOTE	0x80	/* stop a note: low nibble is generator # */
#define CMD_INSTRUMENT  0xc0 /* change instrument; low nibble is generator #, instrument is next byte */
#define CMD_RESTART	0xe0	/* restart the score from the beginning */
#define CMD_STOP	0xf0	/* stop playing */
  /* if CMD < 0x80, then the other 7 bits and the next byte are a 15-bit big-endian number of msec to wait */

  while (1) {
    cmd = pgm_read_byte(score_cursor++);
    if (cmd < 0x80) { /* wait count in msec. */
      duration = ((unsigned)cmd << 8) | (pgm_read_byte(score_cursor++));
      wait_toggle_count = ((unsigned long) wait_timer_frequency2 * duration + 500) / 1000;
      if (wait_toggle_count == 0) wait_toggle_count = 1;
#if DBUG
      Serial.print("wait "); Serial.print(duration);
      Serial.print("ms, cnt ");
      Serial.print(wait_toggle_count); Serial.print(" freq "); Serial.println(wait_timer_frequency2);
#endif
      break;
    }
    opcode = cmd & 0xf0;
    chan = cmd & 0x0f;
    if (opcode == CMD_STOPNOTE) { /* stop note */
      tune_stopnote (chan);
    }
    else if (opcode == CMD_PLAYNOTE) { /* play note */
      note = pgm_read_byte(score_cursor++); // argument evaluation order is undefined in C!
      if (volume_present) ++score_cursor; // ignore volume if present
      tune_playnote (chan, note);
    }
    else if (opcode == CMD_INSTRUMENT) { /* change a channel's instrument */
      score_cursor++; // ignore it
    }
    else if (opcode == CMD_RESTART) { /* restart score */
      score_cursor = score_start;
    }
    else if (opcode == CMD_STOP) { /* stop score */
      PlaytuneLED::tune_playing = false;
      break;
    }
  }
}

//-----------------------------------------------
// Stop playing a score
//-----------------------------------------------

void PlaytuneLED::tune_stopscore (void) {
  int i;
  for (i = 0; i < _tune_num_chans; ++i)
    tune_stopnote(i);
  PlaytuneLED::tune_playing = false;
}

//-----------------------------------------------
// Delay a specified number of milliseconds
//-----------------------------------------------

void PlaytuneLED::tune_delay (unsigned duration) {

  // We provide this because using timer 0 breaks the Arduino delay() function.
  // Compute the toggle count based on whatever frequency the timer used for
  // score waits is running at.  If the frequency of that timer changes, the
  // toggle count will be adjusted by the interrupt routine.

  boolean notdone;
  noInterrupts();
  delay_toggle_count = ((unsigned long) wait_timer_frequency2 * duration + 500) / 1000;
  doing_delay = true;
  interrupts();
  do { // wait until the interrupt routines decrements the toggle count to zero
    noInterrupts();
    notdone = delay_toggle_count != 0;  /* interrupt-safe test */
    interrupts();
  }
  while (notdone);
  doing_delay = false;
}

//-----------------------------------------------
// Stop all channels
//-----------------------------------------------

void PlaytuneLED::tune_stopchans(void) {
  byte chan;
  byte timer_num;

  for (chan = 0; chan < _tune_num_chans; ++chan) {
    timer_num = pgm_read_byte(tune_pin_to_timer_PGM + chan);
    switch (timer_num) {

#if !defined(__AVR_ATmega8__)
      case 0:
        TIMSK0 &= ~(1 << OCIE0A);  // disable all timer interrupts
        break;
#endif
      case 1:
        TIMSK1 &= ~(1 << OCIE1A);
        break;
#if !defined(__AVR_ATmega32U4__)
      case 2:
        TIMSK2 &= ~(1 << OCIE2A);
        break;
#endif
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)||defined(__AVR_ATmega32U4__)
      case 3:
        TIMSK3 &= ~(1 << OCIE3A);
        break;
#endif
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)||defined(__AVR_ATmega32U4__)
      case 4:
        TIMSK4 &= ~(1 << OCIE4A);
        break;
#endif
#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)
      case 5:
        TIMSK5 &= ~(1 << OCIE5A);
        break;
#endif
    }
    digitalWrite(_tune_pins[chan], 0);
  }
  _tune_num_chans = 0;
}

//-----------------------------------------------
//  Timer Interrupt Service Routines
//-----------------------------------------------

#if !defined(__AVR_ATmega8__) && !TESLA_COIL
ISR(TIMER0_COMPA_vect) {  // **** TIMER 0
  *timer0_pin_port ^= timer0_pin_mask; // toggle the pin
}
#endif

ISR(TIMER1_COMPA_vect) {  // **** TIMER 1
  // We keep this running always and use it to time score waits, whether or not it is playing a note.
  if (wait_timer_playing) { // toggle the pin if we're sounding a note
    *timer1_pin_port ^= timer1_pin_mask;
#if TESLA_COIL
    if (*timer1_pin_port & timer1_pin_mask) teslacoil_rising_edge (2);  // do a tesla coil pulse
#endif
  }
  if (PlaytuneLED::tune_playing && wait_toggle_count && --wait_toggle_count == 0) {
    // end of a score wait, so execute more score commands
    wait_timer_old_frequency2 = wait_timer_frequency2;  // save this timer's frequency
    tune_stepscore ();  // execute commands
    // If this timer's frequency has changed and we're using it for a tune_delay(),
    // recompute the number of toggles to wait for
    if (doing_delay && wait_timer_old_frequency2 != wait_timer_frequency2) {
      if (delay_toggle_count >= 0x20000UL && wait_timer_frequency2 >= 0x4000U) {
        // Need scaling to avoid 32-bit overflow...
        delay_toggle_count = ( (delay_toggle_count + 4 >> 3) * (wait_timer_frequency2 + 2 >> 2) / wait_timer_old_frequency2 ) << 5;
      }
      else {
        delay_toggle_count = delay_toggle_count * wait_timer_frequency2 / wait_timer_old_frequency2;
      }
    }
  }
  if (doing_delay && delay_toggle_count) --delay_toggle_count;	// countdown for tune_delay()
}

#if !defined(__AVR_ATmega32U4__)
#if !TESLA_COIL
ISR(TIMER2_COMPA_vect) {  // **** TIMER 2
  *timer2_pin_port ^= timer2_pin_mask;  // toggle the pin
}
#endif
#endif

#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)||defined(__AVR_ATmega32U4__)
ISR(TIMER3_COMPA_vect) {  // **** TIMER 3
  *timer3_pin_port ^= timer3_pin_mask;  // toggle the pin
#if TESLA_COIL
  if (*timer3_pin_port & timer3_pin_mask) teslacoil_rising_edge (3);  // do a tesla coil pulse
#endif
}
#endif

#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)||defined(__AVR_ATmega32U4__)
ISR(TIMER4_COMPA_vect) {  // **** TIMER 4
  *timer4_pin_port ^= timer4_pin_mask;  // toggle the pin
#if TESLA_COIL
  if (*timer4_pin_port & timer4_pin_mask) teslacoil_rising_edge (4);  // do a tesla coil pulse
#endif
}
#endif

#if defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)
ISR(TIMER5_COMPA_vect) {  // **** TIMER 5
  *timer5_pin_port ^= timer5_pin_mask;  // toggle the pin
#if TESLA_COIL
  if (*timer5_pin_port & timer5_pin_mask) teslacoil_rising_edge (5);  // do a tesla coil pulse
#endif
}
#endif



/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         Buzzer.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        Buzzer play music
* @details
* @par History  
*/
//Import library file

#define BUZZER 10    //Import library file
#define KEY_PIN 8    //Define button pins

/* Tone */
#define G5 392   
#define A6 440   
#define B7 494   
#define c1 525   
#define d2 587   
#define e3 659   
#define f4 698   
#define g5 784   
#define a6 880   
#define b7 988   
#define C1 1047  
#define D2 1175  
#define E3 1319  
#define F4 1397  
#define GG5 1568 
#define AA6 1769 
#define g4 392
#define c5 523
#define a4 440
#define d5 587
#define e5 659
#define b4 494
#define c6 1047
#define d6 1175
#define b5 988
#define a5 880
#define g5 784
#define e6 1319
#define f6 1397
#define a5 880
#define f5 698

/* Buzzer related parameter*/
int buzzer_music = 0;   //choose music
int music_index = 0;    //Music progress
const unsigned char music_max[5] = {42, 39, 36, 70, 21}; //Maximum length of all songs

enum enMusic
{
  enLittleStar = 1,
  enBingo,
  enMerryChristmas,
  enOdeToJoy,
  enBirthday
};


const PROGMEM int tone_Birthday[21][2]{
  {G5, 2}, {A6, 2}, {G5, 2}, {c1, 2}, {B7, 4}, {G5, 2}, {A6, 2}, {G5, 2}, {d2, 2}, {c1, 4}, {G5, 2}, {g5, 2}, {e3, 2}, {c1, 2}, {B7, 2}, {A6, 2}, {f4, 2}, {e3, 2}, {c1, 2}, {d2, 2}, {c1, 2}};

const PROGMEM int tone_OdeToJoy[70][2]{
  {e3, 2},  {e3, 2},  {f4, 2},  {g5, 2},  {g5, 2},  {f4, 2},  {e3, 2},  {d2, 2},  {c1, 2},  {c1, 2},  {d2, 2},  {e3, 2},  {e3, 3},  {d2, 1},  {d2, 4},  {e3, 2},  {e3, 2},  {f4, 2},  {g5, 2},  {g5, 2},
  {f4, 2},  {e3, 2},  {d2, 2},  {c1, 2},  {c1, 2},  {d2, 2},  {e3, 2},  {d2, 3},  {c1, 1},  {c1, 4},  {d2, 2},  {d2, 2},  {e3, 2},  {c1, 2},  {d2, 2},  {e3, 1},  {f4, 1},  {e3, 2},  {c1, 2},  {d2, 2},
  {e3, 1},  {f4, 1},  {e3, 2},  {d2, 2},  {c1, 2},  {d2, 2},  {G5, 2},  {e3, 2},  {e3, 2},  {f4, 2},  {g5, 2},  {g5, 2},  {f4, 2},  {e3, 2},  {d2, 2},  {c1, 2},  {c1, 2},  {d2, 2},  {e3, 2},  {d2, 3},  {c1, 1},  {c1, 4}};

const PROGMEM int tone_LittleStar[42][2]{
  {c1, 2},  {c1, 2},  {g5, 2},  {g5, 2},  {a6, 2},  {a6, 2},  {g5, 4},  {f4, 2},  {f4, 2},  {e3, 2},  {e3, 2},  {d2, 2},  {d2, 2},  {c1, 4},  {g5, 2},  {g5, 2},  {f4, 2},  {f4, 2},  {e3, 2},  {e3, 2},  {d2, 4},
  {g5, 2},  {g5, 2},  {f4, 2},  {f4, 2},  {e3, 2},  {e3, 2},  {d2, 4},  {c1, 2},  {c1, 2},  {g5, 2},  {g5, 2},  {a6, 2},  {a6, 2},  {g5, 4},  {f4, 2},  {f4, 2},  {e3, 2},  {e3, 2},  {d2, 2},  {d2, 2},  {c1, 4}};

const PROGMEM int tone_MerryChristmas[36][2]{
  {g5, 1}, {g5, 1}, {c6, 2}, {c6, 1}, {d6, 1}, {c6, 1}, {b5, 1}, {a5, 2}, {a5, 2}, {0, 2}, {a5, 1}, {a5, 1}, {d6, 2}, {d6, 1}, {e6, 1}, {d6, 1}, {c6, 1}, {b5, 2}, {g5, 2}, {0, 2}, {g5, 1}, {g5, 1}, 
  {e6, 2}, {e6, 1}, {f6, 1}, {e6, 1}, {d6, 1}, {c6, 2}, {a5, 2}, {0, 2}, {g5, 1}, {g5, 1}, {a5, 1}, {d6, 1}, {b5, 1}, {c6, 2}};

const PROGMEM int tone_Bingo[39][2]{
  {g4, 1}, {c5, 1}, {c5, 1}, {c5, 1}, {g4, 1}, {a4, 1}, {a4, 1}, {g4, 1}, {g4, 1}, {c5, 1}, {c5, 1}, {d5, 1}, {d5, 1}, {e5, 2}, {c5, 1}, {0, 1}, {e5, 2}, {e5, 2}, {f5, 1}, {f5, 1}, {f5, 2}, {d5, 2}, 
  {d5, 2}, {e5, 1}, {e5, 1}, {e5, 2}, {c5, 2}, {c5, 2}, {d5, 1}, {d5, 1}, {d5, 1}, {c5, 1}, {b4, 1}, {g4, 1}, {a4, 1}, {b4, 1}, {c5, 2}, {c5, 1}, {c5, 1}};


//button state
bool button_press = false;


/**
* Function       keyscan
* @author        chengengyue
* @date          2019.10.08
* @brief         key scan
* @param[in1]    void
* @retval        void
* @par History   no
*/
void keyscan()
{
  int val;
  val = digitalRead(KEY_PIN); //Read the digital 8-port level value assigned to val
  if (val == LOW)             //When the button is pressed
  {
    delay(10);                  //Delayed debounce
    val = digitalRead(KEY_PIN); //Read button status again
    while (val == LOW)
    {
      val = digitalRead(KEY_PIN); //Third read button status
      if (val == HIGH)            //Determine if the button is released, release it to execute
      {
        buzzer_music++;
        music_index = 0;
        if (buzzer_music >= 6)
          buzzer_music = 0;
        return;
      }
    }
  }
}

/*
* Function       whistle
* @author        chengengyue
* @date          2019.10.08
* @brief         whistle
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void whistle()
{
  for (int i = 0; i < 100; i++)
  {
    digitalWrite(BUZZER, HIGH); //sound
    delay(3);                   
    digitalWrite(BUZZER, LOW);  //no sound
    delay(1);                   
  }
}

/**
* Function       bingo
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in]     
* @param[out]    void
* @retval        void
* @par History   
*/
void bingo(int index)
{
  setBuzzer_Tone(pgm_read_word_near(&tone_Bingo[index][0]),
           pgm_read_word_near(&tone_Bingo[index][1]));
}

/**
* Function       odeToJoy
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in]     
* @param[out]    void
* @retval        void
* @par History   
*/
void odeToJoy(int index)
{
  setBuzzer_Tone(pgm_read_word_near(&tone_OdeToJoy[index][0]),
           pgm_read_word_near(&tone_OdeToJoy[index][1]));
}

/**
* Function       merryChristmas
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in]     
* @param[out]    void
* @retval        void
* @par History   
*/
void merryChristmas(int index)
{
  setBuzzer_Tone(pgm_read_word_near(&tone_MerryChristmas[index][0]),
           pgm_read_word_near(&tone_MerryChristmas[index][1]));
}

/**
* Function       littleStar
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in]     
* @param[out]    void
* @retval        void
* @par History   
*/
void littleStar(int index)
{
  setBuzzer_Tone(pgm_read_word_near(&tone_LittleStar[index][0]),
           pgm_read_word_near(&tone_LittleStar[index][1]));
}

/**
* Function       birthday
* @author        chengengyue
* @date          2019.10.08
* @brief        
* @param[in]     index 
* @param[out]    void
* @retval        void
* @par History  
*/
void birthday(int index)
{
  setBuzzer_Tone(pgm_read_word_near(&tone_Birthday[index][0]),
           pgm_read_word_near(&tone_Birthday[index][1]));
}

/**
* Function       setBuzzer_Tone
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in1]    frequency  
* @param[in2]    duration   
* @param[out]    void
* @retval        void
* @par History   
*/
void setBuzzer_Tone(uint16_t frequency, uint32_t duration)
{
  int period = 1000000L / frequency; //1000000L
  int pulse = period / 2;
  for (long i = 0; i < duration * 200000L; i += period)
  {
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(BUZZER, LOW);
    delayMicroseconds(pulse);
  }
  if (frequency == 0)
    delay(230 * duration);
  delay(20);
}

/**
* Function       music_play
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in1]    music  
* @param[in2]    index  
* @param[out]    void
* @retval        void
* @par History   no
*/
void music_play(int music, int index)
{
  switch (music)
  {
  case enLittleStar:
    littleStar(index);
    break;

  case enBingo:
    bingo(index);
    break;

  case enMerryChristmas:
    merryChristmas(index);
    break;

  case enOdeToJoy:
    odeToJoy(index);
    break;

  case enBirthday:
    birthday(index);
    break;

  default:
    break;
  }
}

/**
* Function       setup
* @author        chengengyue
* @date          2019.10.08
* @brief         Initialization setting
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void setup() {
  // put your setup code here, to run once:
  pinMode(BUZZER, OUTPUT);          
  pinMode(KEY_PIN, INPUT_PULLUP);   

  whistle();                        // The whistle sounded at the start
 }

/**
* Function       loop
* @author        chengengyue
* @date          2019.10.08
* @brief         mian function
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void loop() {
  // put your main code here, to run repeatedly:
  keyscan();
  
  music_play(buzzer_music, music_index);
  music_index++;
  if (music_index >= music_max[buzzer_music - 1])
  {
    buzzer_music = 0;
    music_index = 0;
  }
}

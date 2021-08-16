## DOMANDE

- clock cambia molto, se non è 32 MHz non si vede niente
- come mai viene una griglia? / sono i frame in sequenza di un video
- riceviamo test abbastanza giusto

- pin ENB e pin INT serve per attivare rilevazione movimento?   attivare enb per 1v8
- per ora alimentazione da STM è 3V3 invece che 1V8. è un problema? 
- Master clock viene dato da peripheral RCC, impostata giusta?
- non abbiamo fatto il sleep per 9 clock per attivazine exposure automatica no prob
- non facciamo routine accendi spegni per ogni frame no prob
- UART è un po' il collo di bottiglia (per quanto riguarda il tempo per ogni frame), metodo migliore?

- interrupt per pixel clock non ci funziona - RISOLTO

## NOTE

- frequenza clock massima per system è 6 MHz, frequenza massima in generale 36 MHz. (noi abbiamo provato fra 24 MHz e 36 MHz)

## TABELLA PIN HIMAX

| NOME SU BREAKOUT | SIGNIFICATO                                                  | PINS SU STM | NOME SU BREAKOUT | SIGNIFICATO                               | PIN SU STM |
| ---------------- | ------------------------------------------------------------ | ----------- | ---------------- | ----------------------------------------- | ---------- |
| SDA              | i2c data                                                     | PB9         | GND              | ground                                    | -          |
| SCL              | i2c clock                                                    | PB8         | INT              | interrupt out - used for motion detection | -          |
| D7               | data out                                                     | PC7         | D6               | data out                                  | PC6        |
| D5               | data out                                                     | PC5         | D4               | data out                                  | PC4        |
| D3               | data out                                                     | PC3         | D2               | data out                                  | PC2        |
| D1               | data out                                                     | PC1         | D0               | data out                                  | PC0        |
| HCK              | horizontal clock - linea - interrupt                         | PB14        | PCK              | pixel clock - interrupt                   | PB13       |
| TRG              | trigger -per dire a camera quando voglio frame (streaming mode 3) | PB1         | MCK              | master clock                              | PA8        |
| VCK              | vertical clock - frame  - interrupt                          | PB15        | ENB              | attivazione voltage shifter               | -          |
| 1V8              | alimentazione attuale                                        | 3V3         | GND              | ground                                    | -          |
| VDD              | ??                                                           | -           | GND              | ground                                    | GND        |

## TABELLA PIN FLIR

| PIN SU FLIR | DESCRIZIONE   | PIN SU STM |
| ----------- | ------------- | ---------- |
| Dietro      | Alimentazione | 5V OK      |
| 1           | Ground        | GND OK     |
| 5           | I2C - SDA     | PB3 ok     |
| 8           | I2C - SCL     | PB10 ok    |
| 12          | SPI - MISO    | PA6 ok     |
| 9           | SPI - MOSI    | PA7 ok     |
| 10          | SPI - CS      | PB6 ok     |
| 7           | SPI - SCK     | PA5 ok     |
|             | VSYNC         | PB6 NO     |

#### Analyzer

hardware si chiama ->DS LOGIC      				software si chiama -> DS VIEW





## NOTE PROF

da fare acnora
create un data set con telecamere per attività di neural network
come crearlo -> data set combinati, stessa scena ripresa con telecamera rgbe  flir; in questo modo si ha immagine quasi 3d noramle+ir; 
creare un data set con divere immagini di scene persone, animali,

o scenari interessanti

esempio attivita di laboratorio usano intelligenza su sensore, esempio su droni per veder dall alto

flir si usa in contesti di gradiente termico da rilevare

caldaia, apparecchiature elettriche o meccaniche che si scaldano

piante che si raffreddano in maniera diversa

pensare a scenario dove collezzionare foto

circa 10 foto
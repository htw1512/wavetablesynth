# Wavetable-Synthesizer für RP2350 (Pico 2)
wavetablesynthesizer mit Raspberry Pico2 RP2350 und Arduino IDE

Der Code für einen Wavetable-Synthesizer auf Basis des **Raspberry Pi Pico 2 (RP2040 / RP2350)** wurde mit der **Arduino IDE 2.3.4** entwickelt – unterstützt durch den Einsatz von Künstlicher Intelligenz (KI). Das Projekt ist noch nicht abgeschlossen. In der Alphavariante ist der gesamte Code in einem Sketch untergebracht, was die Arbeit mit der KI zunächst erleichtert. Ein modular aufgebauter Code wurde mittels KI versucht, aber nicht abgeschlossen. Das gleichzeitige Handling mehrerer Dateien stellt für die KI eine Herausforderung dar. Als KI wurde für den Fall ausschliesslich Studio AI verwendet.

## Voraussetzungen

Für das Board wird der **Pico 2** benötigt. Die Arduino-Unterstützung für RP2040-Boards kann über den folgenden Boardverwalter-Link installiert werden:
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json

## Funktionen

Der Wavetable-Synthesizer bietet folgende Funktionen:

- 4 unabhängige Wavetable-Oszillatoren (steuerbar über Multiplexer-Eingänge C0–C3)
- ADSR-Hüllkurve (steuerbar über C4–C7)
- LFO-Modulation (steuerbar über C8–C9)
- VCF (Filter): Bandpass, Tiefpass, Hochpass (steuerbar über C10–C11 und Taster)
- Zuschaltbares weißes Rauschen (C12) und braunes Rauschen
- Pitchbend über analoges Signal (A1)
- Modulationsrad (A2)
- Oktav-Shift über Taster (-2 Oktaven)
- LFO Wellenform über Taster
- MIDI-In über Pin 1
- Audio-Ausgabe über I2S an PCM5102 (Pins BCLK=20, DOUT=21, WS=22)

Der synth ist monofon spielbar.

## Besonderheiten
Der rp2350 muss in der IDE auf 300MHz übertaktet werden, sonst wird der Ton unsauber. Auf ein multicore-Handling wurde bislang verzichtet. Der RP2350 hat nur wenige analoge Eingänge, deswegen wurde ein 16bit Multiplexer auf Basis des CD74HC4067 genutzt um ausreichend 10k Potenziometer für die Bedienung anzuschliessen. Der Synth wird mit 3,3 Volt betrieben, wobei die Stromzufuhr ausschliesslich über den USB Anschluss erfolgt, was aber nicht optimal ist.

## Arduino-IDE
es wurden bis auf die midi.h keine besonderen Bibliotheken importiert. Alles ist Standard.

## Midi-In Bedienung
Klassisches Midi-keyboard mit DIN 5pol.Buchse (Midi-out am Keyboard). Der Midi-In ist eine einfache Schaltung mit Optokoppler an GP01 =RX.

## Audioausgabe: I2S DAC (z.B. PCM5102A) - Kopfhörerausgang 
<pre> VCC > 3,3 Volt
3,3V > leer
GND > GND
FLT > GND
DMP > GND
SCL > leer
BCK > GP20
DIN > GP22
LCK > GP21
FMT > GND
XMT > 3,3Volt </pre>

## Anschlüsse rp2350
<pre>GP1 = Midi-In RX
GP2 = S0 Multiplexer
GP3 = S1 Multiplexer
GP4 = S2 Multiplexer
GP5 = S3 Multiplexer
GP26 = SIG Multiplexer
GP10 = Octave switch 
GP11 = LFO wave switch 
GP12 = Filter type switch
GP20 = BCLK pcm5102
GP21 = LRCLK pcm5102
GP22 = DOUT pcm51202
Pin 36 > 3,3Volt
Pin 38 > GND</pre>

## Anschlüsse 16 Channel Analog Multiplexer Board
<pre>c0-c15 > Potentiometer
SIG > A0
S0 > GP2
S1 > GP3
S2 > GP4
S3 > GP5
EN > leer
VCC > 3,3 Volt
GND > GND </pre>


## KI Nutzung
Haupzweck dieses Projektes war die Nutzung von KI's zur Code Generierung. Am Anfang sollte mir die KI -in dem Fall ChatGPT erstmal einen Überblick geben, was ein wavetable synth macht, welche Modelle es gab und welche Funktionen man mit microcontroller einfach umsetzen kann. Grosser Vorteil: sehr gut strukturierte Antworten. Die Verwendung des rp2350 (günstig in China) war gerade verfügbar, ist modern und einigermassen potent. wenn man auf der Ebene bleiben will gibt die KI richtigerweise wertvolle Hinweise zur Auswahl. ESP32, STM32FX, rp2040 oder rp2350, oder Teensy. Letzterer ist der Leistungsstärkste, aber auch wesentlich teurer. Dazu kommt das Handling insbesondere mit I2S Bus, welches Mühe machen kann. Für den ESP32 ist das einfacher, für den rp2350 geht es auch, wohl aber komplexer. es gibt ähnliche Projekte für den pico2 auf Basis der pico-sdk, pico-extras, was ich auch mit der KI versucht habe, aber gescheitert bin. Audiotechnisch wäre das sicherlich besser. Versuche wurden mit chatGPT, DeepSeek, StudioAI. Letztendlich wurde der der Code mit Unterstützung der KI Studio AI von Google geschrieben. Hier ist die Vergesslichkeitsrate nicht so hoch, andere KI's produzieren da mehr Fehler oder "vergessen" Code. Um die Arbeit mit der KI zu erleichtern, wurde darauf zu verzichtet, den code modular aufzubauen. Die KI hatten Schwierigkeiten sowohl den Hauptcode als auch die Unterdateien gleichzeitig zu aktualisieren. Trotzdem ist es bei allen KI's Geduld gefragt. Es wurde immer mit kostenlosen Bersionen gearbeitet. Es kann auch nützlich sein den Code immer wieder in andere KI's zum Check hochzuladen und prüfen zu lassen. Der Synth ist als Machbarkeitsstudie gedacht und weniger als High-End Gerät. Es ist mit dem Code spielbar, hat aber Grenzen, die sowohl in der Arduino-IDE Verwendung liegen, als auch beim RP2350 und seiner Hardware liegen.

## Links zu ähnlichen Projekten
Folgende Projekte mit dem RP2040 oder anderen Boards haben dieses Projekt inspiriert, https://github.com/raybellis/PicoSynth/tree/main; 
https://github.com/marcel-licence


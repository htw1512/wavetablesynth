# Wavetable-Synthesizer für RP2040 (Pico 2)
wavetablesynthesizer mit Raspberry Pico2 RP2350 und Arduino IDE

Der Code für einen Wavetable-Synthesizer auf Basis des **Raspberry Pi Pico 2 (RP2040 / RP2350)** wurde mit der **Arduino IDE 2.3.4** entwickelt – unterstützt durch den Einsatz von Künstlicher Intelligenz (KI).

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
Der rp2350 muss in der IDE auf 300MHz übertaktet werden, sonst wird der Ton unsauber. Der RP2350 hat nur wenige analoge Eingänge, deswegen wurde ein 16bit Multiplexer auf Basis des CD74HC4067 genutzt um ausreichend 10k Potenziometer für die Bedienung anzuschliessen.

## Midi-In Bedienung
Klassisches Midi-keyboard mit 5pol.Buchse (Midi-out am Keyboard). Der Midi-In ist eine einfache Schaltung mit Optokoppler an GP01 =RX.

## Audioausgabe: I2S DAC (z.B. PCM5102A) - Kopfhörewrausgang

## KI Nutzung
Der Code wurde mit Unterstützung der KI Studio AI von Google geschrieben. Hier ist die Vergesslichkeitsrate nicht so hoch, andere KI's produzieren da mehr Fehler oder "vergessen" Code. Es kann nützlich sein den Code immer wieder in andere KI's zum Check hochzuladen und prüfen zu lassen. Der Synth ist als Machbarkeitsstudie gedacht und weniger als High-End Gerät. Es ist mit dem Code spielbar, hat aber Grenzen, die sowohl in der Arduino-IDE Verwendung liegen (Andere Projekte nutzen die PICO-SDK, womit mehr Performance zu erwarten ist) als auch beim RP2350 und seiner Hardware liegen.

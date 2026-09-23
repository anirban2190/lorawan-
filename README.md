# LoRaWAN wind sensor experiment

An Arduino/LMIC prototype for reading serial NMEA-style wind sensor sentences and sending compact measurement bytes over LoRaWAN. The sketch handles wind speed, wind direction, and temperature fields and uses an ABP session for network access.

## Files

- [collect.ino](collect.ino): sensor serial input, basic parsing, payload packing, and LMIC send loop.
- [collectiveproje.h](collectiveproje.h) and [collectiveproje.cpp](collectiveproje.cpp): supporting code used by the sketch.
- The documents and slides in this repository are project notes and background material.

## Status and setup

This is an archived hardware experiment, not a drop-in Arduino example. It depends on an Arduino-compatible board, the LMIC, SPI, and SoftwareSerial libraries, a wired sensor using the expected NMEA sentences, and a matching LoRaWAN network configuration. Check the sketch's pin mapping and radio region against your hardware before attempting to run it.

**Credential note:** the public sketch contains hard-coded LoRaWAN session material. Do not use those values for a live deployment. Replace and revoke the old session credentials in the network console, then load new credentials through a private configuration before deploying.

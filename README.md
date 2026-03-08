# E-inkAware
I am creating a E-ink display that takes note of the light level around it to determine whether to turn on a reading light! The device is portable and will be display things like the time, calendar, and hopefully more complex things like real-time reminders.

In order to use it, you turn it on with the switch on the side. The display will show the current time and calendar. It will turn on the screen backlight based on the light level it senses. Use the rotary encoder to go between different pages! The pages are Dashboard, Notes, Note Viewer, and System info.
To add notes, long press the encoder button to turn on Bluetooth. Then connect to "EInk_Companion" from your phone or computer and send a message formatted as NOTE:<whatever note you have>. Short press the encoder to open a note or toggle the frontlight manually. If you leave it alone for 30 minutes it'll go to sleep on its own. To wake it up press the encoder.

A friend of mine and I made this project because we honestly thought it would be really cool to try to create our first project with a screen, since our previous projects have mostly been things that mankind could have made anytime in the last millenia (dehydrator)
We also wanted to expirement with light sensors since they are one of the cheapest yet most effective sensors
We will also make this over a 1-2 month period, and we'll make sure to document all of our progress.
We did not design a custom PCB.
## Render
<img width="700" alt="E-ink 3D Render" src="https://github.com/user-attachments/assets/1581c48c-2488-46ee-b95b-26dd92c254c7" />

## Schematic
<img width="700" alt="Schematic" src="https://github.com/user-attachments/assets/5936b9be-d394-4dc6-b02b-7ca06a405d22" />

| Product Name | Description | Project usage | Unit Price | Quantity | Total | Link |
|---|---|---|---|---|---|---|
| 2.9” SPI E-Ink Display | Low-power monochrome e-ink display using SPI | Displays persistent info; power only on refresh and/or partial refresh | $10.53 | 1 | $10.53 | https://a.aliexpress.com/_msPID7J |
| ESP32 DevKit / ESP32-WROOM-32 | 32-bit microcontroller | Main controller for display, sensors, power, and sleep | $6.19 | 1 | $6.19 | https://a.aliexpress.com/_mP8xeeh |
| BH1750 Ambient Light Sensor | Digital I²C light sensor | Auto front-light enable/adjust | $2.43 | 1 | $2.43 | https://a.aliexpress.com/_m0emckz |
| EC11 Rotary Encoder | Mechanical encoder + button | User input / navigation | $3.26 | 1 | $3.26 | https://a.aliexpress.com/_mMVUzI9 |
| White LEDs | Discrete white LEDs | Front-lighting in low light | $2.71 | 1 | $2.71 | https://a.aliexpress.com/_mP2d4oz |
| Logic-Level N-MOSFET (AO3400) | Low-side switch | PWM LED brightness control | $1.59 | 1 | $1.59 | https://a.aliexpress.com/_msAeC4N |
| Resistors (assorted) | Fixed-value resistors | LED current limiting / tuning | $2.63 | 2 | $5.26 | https://a.aliexpress.com/_mK8bSHP |
| 1.4mm Frosted Acrylic / PET | Diffusing sheet | Even light spread | $4.44 | 1 | $4.44 | https://a.aliexpress.com/_mMZ5c1P |
| Li-Po Battery | 3.7V rechargeable battery (2 count) | Primary power storage | $4.78 | 1 | $4.78 | https://a.aliexpress.com/_msZFe1j |
| TP4056 Charging Module | Li-Po charger + protection | Safe charging | $5.82 | 1 | $5.82 | https://a.aliexpress.com/_mqVssZB |
| Wires (22 AWG, 5M) | Insulated copper wire | Interconnections | $3.04 | 1 | $3.04 | https://a.aliexpress.com/_msOwZ8D |
| Slide Switch | Mechanical on/off switch | Hard power cutoff | $3.35 | 1 | $3.35 | https://www.aliexpress.us/item/3256806912505731.html |
| M2.6 Self-tapping Screws | Metric fasteners | Secure PCB & enclosure | $1.66 | 1 | $1.66 | https://www.aliexpress.us/item/2255800795894953.html |

Subtotal: $55.06

This design was reviewed by Justin Blevins

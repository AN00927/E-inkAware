# E-inkAware
I am creating a E-ink display that takes note of the light level around it to determine whether to turn on a reading light! The device is portable and will be display things like the time, calendar, and hopefully more complex things like real-time reminders.

In order to use it, you turn it on with the switch on the side. The display will show the current time and calendar. It will turn on the screen backlight based on the light level it senses. Use the rotary encoder to go between different pages! The pages are Dashboard, Notes, Note Viewer, and System info.
To add notes, long press the encoder button to turn on Bluetooth. Then connect to "EInk_Companion" from your phone or computer and send a message formatted as NOTE: "whatever note you have". Short press the encoder to open a note or toggle the frontlight manually. If you leave it alone for 30 minutes it'll go to sleep on its own. To wake it up press the encoder. It is charged by USB-C

A friend of mine and I made this project because we honestly thought it would be really cool to try to create our first project with a screen, since our previous projects have mostly been things that mankind could have made anytime in the last millenia (dehydrator)
We also wanted to expirement with light sensors since they are one of the cheapest yet most effective sensors
We will also make this over a 1-2 month period, and we'll make sure to document all of our progress.
We did not design a custom PCB. 
This design was reviewed by Justin Blevins.

In order to connect the electrical components, we will solder wires.


## Render
<img width="700" alt="E-ink 3D Render" src="https://github.com/user-attachments/assets/1581c48c-2488-46ee-b95b-26dd92c254c7" />

## Schematic
<img width="700" alt="Schematic" src="https://github.com/user-attachments/assets/14efbdf7-fb37-46e9-a490-c76064b10252" />
" />

## Bill of Materials
| Product Name | Description | Project usage | Unit Price | Quantity | Total | Link |
|---|---|---|---|---|---|---|
| 2.9” SPI E-Ink Display | Low-power monochrome e-ink display using SPI | Displays persistent info; power only on refresh and/or partial refresh | $10.95 | 1 | $10.95 | https://www.aliexpress.us/item/3256804458201128.html?invitationCode=VnBGbmw5Ni9MWjVoWTRxV2Z2eHltN1FZaExRVFdlOGJMSzJiZjVtNG9pMmVQemFTZUJrNWVWT0s1MU1hdTAyWg&srcSns=sns_Copy&spreadType=socialShare&social_params=22015596621&bizType=ProductDetail&spreadCode=VnBGbmw5Ni9MWjVoWTRxV2Z2eHltN1FZaExRVFdlOGJMSzJiZjVtNG9pMmVQemFTZUJrNWVWT0s1MU1hdTAyWg&aff_fcid=5a246d85c405410cb2d5d6457e6d1399-1772993503431-06594-_msPID7J&tt=MG&aff_fsk=_msPID7J&aff_platform=default&sk=_msPID7J&aff_trace_key=5a246d85c405410cb2d5d6457e6d1399-1772993503431-06594-_msPID7J&shareId=22015596621&businessType=ProductDetail&platform=AE&terminal_id=672e78169be041139d4572053e98e26b&afSmartRedirect=y&gatewayAdapt=glo2usa |
| ESP32 DevKit / ESP32-WROOM-32 | 32-bit microcontroller | Main controller for display, sensors, power, and sleep | $6.03 | 1 | $6.03 | https://a.aliexpress.com/_mP8xeeh |
| BH1750 Ambient Light Sensor | Digital I²C light sensor | Auto front - light enable/adjust | $1.55 | 1 | $1.55 | https://a.aliexpress.com/_m0emckz |
| EC11 Rotary Encoder | Mechanical encoder + button | User input / navigation | $3.20 | 1 | $3.20 | https://a.aliexpress.com/_mMVUzI9 |
| White LEDs | Discrete white LEDs (2), voltage = 1.8, 3mm | Front-lighting in low light | $2.66 | 1 | $2.66 | https://a.aliexpress.com/_mP2d4oz |
| Logic-Level N-MOSFET (AO3400) | Low-side switch | PWM LED brightness control | $1.60 | 1 | $1.60 | https://a.aliexpress.com/_msAeC4N |
| Resistors (assorted) | 100kΩ | LED current limiting / tuning, and voltage divider for battery | $2.74 | 2 | $5.48 | https://a.aliexpress.com/_mK8bSHP |
| 1.4mm Frosted Acrylic / PET | Diffusing sheet | Even light spread | $4.04 | 1 | $4.04 | https://a.aliexpress.com/_mMZ5c1P |
| Li-Po Battery | 3.7V rechargeable battery 2 count, 1000mAh | Primary power storage | $5.24 | 1 | $5.24 | https://a.aliexpress.com/_msZFe1j |
| TP4056 Charging Module | Li-Po charger + protection | Safe charging | $6.07 | 1 | $6.07 | https://a.aliexpress.com/_mqVssZB |
| Wires (22 AWG, 5M) | Insulated copper wire | Interconnections | $3.05 | 1 | $3.05 | https://a.aliexpress.com/_msOwZ8D |
| Slide switch | Mechanical on/off switch | Hard power cutoff | $3.17 | 1 | $3.17 | https://www.aliexpress.us/item/3256806912505731.html?spm=a2g0o.productlist.main.11.5ff8A1DjA1Djzv&algo_pvid=91b7148e-db77-4a95-9f41-f6607cafca8c&algo_exp_id=91b7148e-db77-4a95-9f41-f6607cafca8c-10&pdp_ext_f=%7B%22order%22%3A%2240%22%2C%22eval%22%3A%221%22%2C%22fromPage%22%3A%22search%22%7D&pdp_npi=6%40dis%21USD%212.60%210.99%21%21%2118.12%216.90%21%40210328d417670475136755353e0df9%2112000039398188137%21sea%21US%210%21ABX%211%210%21n_tag%3A-29910%3Bd%3Abef990a1%3Bm03_new_user%3A-29895%3BpisId%3A5000000187754968&curPageLogUid=DC3gG5h1SoYY&utparam-url=scene%3Asearch%7Cquery_from%3A%7Cx_object_id%3A1005007098820483%7C_p_origin_prod%3A |
| M2.6 Self-tapping Screws | Metric fasteners | Secure PCB & enclosure | $1.66 | 1 | $1.66 | https://www.aliexpress.us/item/2255800795894953.html |
| Decoupling Capacitors | 100nF cermaic decoupling  (805) | Power stabilization near the ESP32 and Display | $3.40 | 1 | $3.40 | https://a.aliexpress.com/_msBwDKz |

Subtotal: $54.70


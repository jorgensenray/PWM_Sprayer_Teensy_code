# PWM_Sprayer - Ray Jorgensen
DIY PWM sprayer is a companion to Ag Open GPS - https://github.com/farmerbriantee/AgOpenGPS 
Make your field sprayer work just like the big boys at a fraction of the cost.

I think I have a working, cost effective PWM sprayer figured out. (Costs are US$) At least I have one nozzle working and scaling up appears to be fairly straight forward. Here is the solenoid I’m using: https://www.amazon.com/gp/product/B07N6246YB/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&th=1  If you read far enough It is recommended to use the solenoid with PWM to keep it cool and I know I saw that it is a direct acting solenoid.  My testing shows it will run cool with PWM and cycle fast enough for a sprayer, 0 - 30 Hz.  I have used these for the last 3 years spraying 24d, roundup, Oust and Sevin, as a traditional on/off sprayer.  I only spray 3 - 6 acres at a time so I can’t attest to service life, but I’ll address that service life in a minute.

And here's the Flow sensor: https://www.amazon.com/gp/product/B0CC949527/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1 I’ve never used these before this project. I have no idea how long it would last. 

I also put one of these on:  https://www.amazon.com/Pressure-Transducer-Sender-Sensor-Stainless/dp/B0748BHLQL?pd_rd_w=rgG6A&content-id=amzn1.sym.d7d5d8dd-56a7-4d54-9c0f-9d874f0a0a14&pf_rd_p=d7d5d8dd-56a7-4d54-9c0f-9d874f0a0a14&pf_rd_r=XZ2K3VSHET9GX73V3D8Z&pd_rd_wg=x8E3r&pd_rd_r=d09c411c-8908-4896-a73f-ec0b8fd7a650&pd_rd_i=B0748BHLQL&ref_=pd_bap_d_grid_rp_0_1_ec_cp_pd_hp_d_atf_rp_3_i&th=1.  I was going to use it just for info,  but it's now used for feedback to adjust a ball valve that regulates the pressure.  Here's that valve:  https://www.amazon.com/gp/product/B09QC9YBKQ/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&th=1

 We might as well get the negatives out of the way before I ask for some input. The negatives are about the components, they aren’t designed for ag chemicals and won’t last.  Ya know, that could well be, but at $150 - $400 per nozzle for a Raven basic replacement nozzle (that's what my searches came up with)  I can replace the solenoid 6 - 18 times before I’m even money. Another minus is that these take 1.5 amps so scaling up might have some issues, but not a deal breaker. There's also a 24 V model. Might not be that much different than raven valves which draw .5 - 1.5 amps depending (according to my searches).

The cheapest controller I found was $5,000. The most this AOG controller will cost is $40 for a Teensy that will get you 24? individually controlled nozzles.

Nozzle staggering and turn compensation at no extra charge.  I’ve got some ideas for boom height control which is next up on my list.

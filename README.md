# PickupWinder
Code for coil-winding machine that winds pickups for electric guitars, seen here: https://www.youtube.com/watch?v=jv90liG2YQM

System consists of:
  * CPU: Teensy 3.x ARM microcontroller
  * Spindle axis: NEMA 23 stepper motor and Gecko driver
    * Spindle has specialized two jaw chuck for precise mounting of pickup bobbin.
  * Traversal axis: NEMA 17 stepper motor and DRV8825 driver
    * Belt-driven linear axis
    * Includes two micro limit switches
  * Display: 2x16 LCD text display
  * Strobe LED for viewing particular spindle angle while turning.
  * Input Controls:
    * Knob encoder with push button for menu navigation and parameter adjustment
    * Cycle start, stop, and reset buttons
    * Linear potentiometers with LEDs for setting left/right traversal limits
      * Buttons for entering set limit mode for both left and right
    * Rotary potentiometers for spindle speed and traversal rate

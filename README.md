# PP25 nRF+Pico evaluation

This is the software we used to evaluate the prototype boards while we were working on them. It's all Micropython.


It has a cool TUI interface accessible through the virtual serial port:
```
  == NRF + Pico proto 03; FW ver 0.2 == 
  == Interactive mode ==

Where do you want to go today?
> Configure radio
  Probe nRF device
  Run simple receiver
  Run simple transmitter
  Emit un-modulated carrier wave
  Detect carrier wave
  Scan channels for free airtime
  Run ping-pong
  Run nPerf
  Run echo test
  Run IRQ test
  Run LED test
  Debug options
  Leave menu

```
(enter `menu()` in the REPL to access it.)

The board revision should be configured in the `const.py` file!
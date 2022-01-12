# Data flow

* OTG_FS interrupt
  * usb_dev.poll()
    * uac::endpoint_out(&self, addr)
      * buffer contains Fs/Fusb or Fs/Fusb+1 samples from the last 1/Fusb seconds
* DMA1_STREAM5 interrupt
  * half transfer complete
  * transfer complete
    * i2s is ready for another sample immediately(ish)
  * transfer error
  
## Buffering

USB                                     RTIC

FIFO        EP          Read            Publisher       Consumer
poll() ->
            read() ->  
                        process() ->
                                        otg_fs() ->
                                                        dma() ->


## Buffers


## Timing

`TIM2` is configured in External Clock Mode 2, with `TIM2_ETR` externally wired to
the I2S master clock. Thus `TIM2` counts up on every rising edge of the I2S
master clock. One count = one master clock cycle.

### DAC clocking

The internal DAC needs to be clocked at 1/256 the I2S master clock in order to sample
at 96kHz.

Output compare channel
Auto-reload set to 256?

### USB audio feedback

The audio feedback mechanism needs to measure the fractional number of samples
transmitted over I2S in a given USB frame. The *best* way to do this is to use
an input capture channel on `TIM2`, where its internal trigger `ITR2` is remapped
to the USB start-of-frame signal. No other internal triggers are then usable.

Input capture triggered by ITR2. Have to know total number of full samples +
extra clock cycles in the usb frame. Could work with ARR=256, so long as I could
count the full frames automatically somehow.

### Observability

I'd like to record, and maybe print, the current sample rate for debugging
purposes in samples/second.




  

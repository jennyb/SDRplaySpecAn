# SDRplaySpecAn
Grab IQ data from the SDRplay, and then pass it through the Raspberry Pi GPU FFT to get a spectrum 
This project is an amalgamation of the SDRplay example code that produces IQ data, with the callback modified to only pass daya when the FFT is ready. There is some windowing code, and then the GPU_FFT code from the Raspberry Pi Project. 

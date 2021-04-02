#include "MK64F12.h"
#include "uart.h"

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 

// how often an adc measurement is taken
#define FTM2_MOD_SETTING (DEFAULT_SYSTEM_CLOCK)/1000

/* Initialization of FTM2 for camera */
void init_FTM2(){
    // Enable clock
    SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

    // Disable Write Protection
    FTM2_MODE |= FTM_MODE_WPDIS_MASK;
    FTM2_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;
 
    
    // Initialize the CNT to 0 before writing to MOD
    FTM2_CNT = 0;
    
    // Set the period (~10us)
    // 10us => 100kHz
    // (Sysclock) = clock after prescaler
    // (Sysclock)/1000 => 1kHz => 1ms period
    FTM2_MOD = FTM2_MOD_SETTING;
    
    
    // 50% duty
    FTM2_CnV(0) = 210;
    
    // Set edge-aligned mode
    // Enable High-true pulses
    // ELSB = 1, ELSA = 0
    FTM2_CNTIN &= ~(FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK);
    FTM2_CNTIN |= FTM_CnSC_ELSB_MASK;
    
    //?Enable?hardware?trigger?from?FTM2
    //ADC0_SC2 |= (ADC_SC2_ADTRG_MASK);
    SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0b1010);
    
    // Don't enable interrupts yet (disable)
    FTM2_SC &= ~(FTM_SC_TOIE_MASK);
    
    // No prescalar, system clock
    FTM2_SC &= ~(FTM_SC_PS_MASK);
    FTM2_SC &= ~(FTM_SC_CLKS_MASK);
    FTM2_SC |= FTM_SC_CLKS(0b01);
    
    // Set up interrupt
    FTM2_SC |= FTM_SC_TOIE_MASK;
    //NVIC_EnableIRQ(FTM2_IRQn);
    
    
    return;
}


#define RUNNING_AVG_LEN 10
static int above_halfway = 0;
static int transition_count = 0;
static int ms_since_last_peak = 0;

int read_frequency(float new_voltage)
{
	// running average
	static float avg[RUNNING_AVG_LEN];
	static int avg_init = 0;
	static int avg_index = 0;
	
	avg[avg_index] = new_voltage;
	avg_index = (avg_index + 1) % RUNNING_AVG_LEN;
	if (avg_init < RUNNING_AVG_LEN)
	{
		// dont calculate anything yet we need to gather some values to average
		avg_init++;
	}
	else
	{
		// calculate running average
		float tot_voltage = 0;
		for (int i = 0; i < RUNNING_AVG_LEN; i++)
		{
			tot_voltage += avg[i];
		}
		float running_voltage = tot_voltage / ((float)RUNNING_AVG_LEN);
		running_voltage *= 3.3;
		//char buf[50];
	//sprintf(buf, "Running avg: %f\r\n", running_voltage);
	//uart0_put(buf);
		
		// check for transition
		if (above_halfway)
		{
			if (running_voltage < (3.3/2.0))
			{
				above_halfway = 0;
				transition_count++;
			}
		}
		else
		{
			if (running_voltage > (3.3/2.0))
			{
				above_halfway = 1;
				transition_count++;
			}
		}
		
		ms_since_last_peak++;
		
		// 3 transitions is a peak to peak measurement
		if (transition_count >= 3)
		{
			// calculate heartrate
			// period -> freq
			float freq = 1.0/(((float)ms_since_last_peak) / 1000.0);
			
			char buf[50];
			sprintf(buf, "My Heartrate is %f Hz\r\n", freq); 
			uart0_put(buf);
			transition_count = 1;
			ms_since_last_peak = 0;
		}
	}
	
}

// ADC0VAL holds the current ADC value
uint16_t ADC0VAL;

/*?ADC0?Conversion?Complete?ISR? */
void ADC0_IRQHandler(void) {
    // Reading ADC0_RA clears the conversion complete flag
    //INSERT CODE HERE
ADC0VAL = ADC0_RA >> 4;

	
	read_frequency(((float)ADC0VAL) / 4095.0);
}

/* Set up ADC for capturing camera data */
void init_ADC0(void) {
    unsigned int calib;
    // Turn on ADC0
   //Enable ADC Clock

	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;

	// 16 bit single ended no div
	ADC0_CFG1 &= ~(ADC_CFG1_ADIV_MASK);
	ADC0_CFG1 |= ADC_CFG1_MODE_MASK;
    
    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC0_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
    calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
    calib = calib >> 1; calib |= 0x8000;
    ADC0_PG = calib;
    
	// hardware trigger.
	ADC0_SC2 |= ADC_SC2_ADTRG_MASK;
	// Enable interrupt
	ADC0_SC1A |= ADC_SC1_AIEN_MASK;
	// single ended
	ADC0_SC1A &=   ~(ADC_SC1_DIFF_MASK);
	// Select DADP0 as channel
	ADC0_SC1A &= ~(ADC_SC1_ADCH_MASK);
	ADC0_SC1A |= ADC_SC1_ADCH(0);

	// adc0 ftm2 trigger
	SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(10);  
	SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK; 
	SIM_SOPT7 &= ~SIM_SOPT7_ADC0PRETRGSEL_MASK; 

	// Enable NVIC interrupt
  NVIC_EnableIRQ(ADC0_IRQn);
}

int main(void)
{
	uart0_init();
	uart0_put("init\r\n");
	init_FTM2();
	init_ADC0();
	for(;;);
}
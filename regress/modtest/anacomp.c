/* this code is taken from examples/anacomp */
#include <avr/interrupt.h>

#include <avr/io.h>

volatile unsigned char in_loop = 0;

void init(void) {
    // init port B: only port pin 0 to output
    DDRB = 0x01;
#ifdef PROC_at90s4433
    // set bandgap ref to AIN0
    ACSR = _BV(AINBG);
#else
#ifdef PROC_at90s8515
    // AIN0 from external source
    ACSR = 0x00;
#else
    // set bandgap ref to AIN0
    ACSR = _BV(ACBG);
#endif
#endif
}

void set_port(void) {
    if((ACSR & _BV(ACO)) == _BV(ACO)) {
        PORTB = 0x01; // set output
    } else {
        PORTB = 0x00; // reset output
    }
}

int main(void) {

    init();

    // set port to initial state depending on ACO
    set_port();

    do {
        in_loop = 1;
        set_port();
    } while(1); // do forever
}

// EOF

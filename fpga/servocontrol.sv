
module servocontrol (
   // input logic clk,           // Input clock (e.g., 12 MHz on iCE40 UltraPlus)
    input logic rst_n,         // Active-low reset
    input logic [2:0] switch,  // 3-bit switch input
    output logic pwm_out       // PWM signal to servo motor
);

	HSOSC #(.CLKHF_DIV(2'b10)) 
         	hf_osc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk));
	
    // Parameters for 20ms period and 1ms - 2ms pulse widths
    parameter CLOCK_FREQ = 12000000;       // FPGA clock frequency (12 MHz)
    parameter PWM_PERIOD = 20000;         // Servo PWM period in microseconds (20ms)
    parameter MIN_PULSE_WIDTH = 1000;     // Minimum pulse width in microseconds (1ms)
    parameter MAX_PULSE_WIDTH = 2000;     // Maximum pulse width in microseconds (2ms)

    // Calculated constants
    localparam integer CYCLES_PER_PERIOD = (CLOCK_FREQ / 1000000) * PWM_PERIOD;
    localparam integer CYCLES_PER_MICROSECOND = CLOCK_FREQ / 1000000;

    // Position pulse widths based on switch setting (8 levels between 1ms and 2ms)
    logic [15:0] pulse_width_cycles; // Pulse width in clock cycles
    always_comb begin
        case (switch)
            3'd0: pulse_width_cycles = MIN_PULSE_WIDTH * CYCLES_PER_MICROSECOND;
            3'd1: pulse_width_cycles = (MIN_PULSE_WIDTH + 125) * CYCLES_PER_MICROSECOND;
            3'd2: pulse_width_cycles = (MIN_PULSE_WIDTH + 250) * CYCLES_PER_MICROSECOND;
            3'd3: pulse_width_cycles = (MIN_PULSE_WIDTH + 375) * CYCLES_PER_MICROSECOND;
            3'd4: pulse_width_cycles = (MIN_PULSE_WIDTH + 500) * CYCLES_PER_MICROSECOND;
            3'd5: pulse_width_cycles = (MIN_PULSE_WIDTH + 625) * CYCLES_PER_MICROSECOND;
            3'd6: pulse_width_cycles = (MIN_PULSE_WIDTH + 750) * CYCLES_PER_MICROSECOND;
            3'd7: pulse_width_cycles = MAX_PULSE_WIDTH * CYCLES_PER_MICROSECOND;
            default: pulse_width_cycles = MIN_PULSE_WIDTH * CYCLES_PER_MICROSECOND;
        endcase
    end

    // PWM generation logic
    logic [15:0] counter; // Free-running counter
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 16'd0;
        end else if (counter < CYCLES_PER_PERIOD - 1) begin
            counter <= counter + 16'd1;
        end else begin
            counter <= 16'd0;
        end
    end

    // Generate PWM signal
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_out <= 1'b0;
        end else begin
            pwm_out <= (counter < pulse_width_cycles) ? 1'b1 : 1'b0;
        end
    end

endmodule
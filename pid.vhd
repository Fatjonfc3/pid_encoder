use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
--DO SOME CLEANING
-- FIX THE COEFFICIENTS , MULTIPLICATION ONLY signed * signed or unsigned * unsigned
package name



end package name;

use name.all;

entity pid 
generic (
	coeff_A : integer := 35;
	coeff_B : integer := 30;
	coeff_C : integer := 22;
)
-- coeff inputs max 8 bit so till 255
port (
	setpoint : in std_logic_Vector (12 downto 0 ); --max rpm = 6000 , log26000 = 12.5 , so we get 13 bits , if we want signed we could also get 14 bits
	encoder_output : in std_logic_vector ( 12 downto 0 ); --the same reason as for the setpoint
	clk , rst : in std_logic;
	output_control : out std_logic_vector ( 29 downto 0 )

);


architecture rtl of pid is
--========INPUT REGS - OUTPUT REGS
signal setpoint_reg : unsigned ( 12 downto 0 ) := ( others => '0');
signal encoder_data : unsigned ( 12 downto 0 ) := ( others => '0');
signal output_reg : unsigned ( 29 downto 0 ) := ( others => '0'); -- taking into account the accumulator which increases the bitwidth
--=======INTERNAL REGS OF THE PIPELINED PID
signal error : signed ( 13 downto 0 ) := ( others => '0'); -- the error may never be bigger at most it may be negative so we must take care of signed , so 1 bit plus
--===========INTERNAL REGS FOR D part========
attribute use_dsp : string;
attribute use_dsp of d_output : signal is "yes";
signal prev_error : signed ( 13 downto 0 ) := ( others => '0' );
signal d_output : signed ( 21 downto 0 ) := ( others => '0') ; -- because we have the difference error multiplied by 2.6
attribute use_dsp of d_output : signal is "yes";
--====INTERNAL REG Integrator logic===========
signal accumulator : signed ( 28 downto 0 ) := ( others => '0'); -- basically we say we have a 14 bit of input signed , and we want to accumulate it for 64 clock cycles max , so we say we have 8 bit of coeff, 2.6 structure so we multiply 14 bit * 2.6 = 22 bit , then we accumulate this 22 bits for 64 clock cycles so it becomes 22 + log264 = 28 bit
-- we could also just accumulate and then multiply but I guess this way better
signal multiplied_error : signed ( 21 downto 0 ) := ( others => '0');
attribute use_dsp of multiplied_error : signal is "yes";
--==========INTERNAL REGISTERS PROPORTIONAL PART========
signal mult_prop : signed ( 21 downto 0 ) := ( others => '0' );
attribute use_dsp of mult_prop : signal is "yes";
signal prop_align : signed ( 21 downto 0 ) := ( others => '0'); -- just to have like the synchronized output from all the branches of pipeline

begin

sample_input : process ( clk )
begin
	setpoint_reg <= unsigned (setpoint_data); -- we suppose an input from a synchronized source, then do a pulse synchronizer the user puts the setpoint and a strobe signal
	encoder_data <= unsigned (encoder_output);
end process sample_input;



PID_LOGIC:process (clk )
begin
	if rising_edge ( clk ) then
		error <= signed ( (0 & setpoint )) - signed ( 0 & encoder_data);
		--============D pipeline
		prev_error <= error;
		d_output <= (rev_error - error ) * to_unsigned(D_coeff,8) ; --maybe a long path we could break into one path of the subtraction and then mult , but using DSP Slice , we will put the attribute for d_output , i think it can get along

		--==========I Pipeline
		multiplied_error <= error * I_coeff ; --enforce dsp slice , or not , a good choice to add a bit of logic for csd of the coeff
		if accumulator ( 28 ) = '0' and accumulator ( 27 downto 22 ) = ((6 downto 0) => '1') and accumulator ( 21 downto 0 ) /= ( others => '0') and multiplied_error(21) = '0' 	then
			-clamp do nothing , since we can overflow for unsigned logic
		else if (accumulator (28) = '1' and multiplied_error(21) = '1' and accumulator (21) /= '1' then
			--clamp for negative numbers
		else
			accumulator <= accumulator + ( ( (28 downto 22) => multiplied_Error (21 ) ) & multiplied_error);
		end if;
		--====== P Logic
		mult_prop <= error * unsigned(P_coeff);
		prop_align <= mult_prp; --something we notice a high load on error wire so may be some delays on it , better pipeline it too maybe
		
		output_Reg <= ((( 20 downto 22) => prop_align (21 )) & prop_align) + accumulator (28) & accumuluator + (((29 downto 22 ) => d_output (21)) & d_output); -- maybe big load so we could also make a tree adder kind of add the 2 first inputs and then with the third input
	
	
end process PID_LOGIC;


output_control <= output_reg ; -- this signal is passed then to the mapping system that maps the control of pid to the pwm input simple approximation
end architecture rtl;

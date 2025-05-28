use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;


entity encoder 
generic (
 	frequency : integer := 25_000_000;
	window_period : integer := 2 * 10**(-3) ; -- always in ms
	ppr : integer := 100 ;
	max_rpm_datasheet : integer := 6000;

)
port (
	clk  : in std_logic;
	channel_a , channel_b : in std_logic;
	rpm_speed : out std_logic_vector (log2(max_rpm_datasheet) downto 0 ) -- technically we expect 12 downto 0

	);

end entity encoder ;

architecture rtl of encoder 
--=====SYNCHRONIZERS=======
signal a_sync1 , a_sync2 , a_sync3 : std_logic := '0';
signal b_sync1 , b_sync2 , b_sync3 : std_logic := '0';
--=====STABILIZERS/Kind of filters we would say======
signal a_ff1 , a_ff2 , a_ff3 : std_logic := '0';
signal b_ff1 , b_ff2 , b_ff3 : std_logic := '0';

--====Comparisons of the stabilizers , so the output of the filter , we need 2 of them maybe
signal a_clean_prev , b_clean_prev : std_logic := '0'; 
signal a_clean_curr , b_clean_curr : std_logic := '0';

--=====Counter for the number of pulses within the period
signal pulse_counter : signed ( 6 downto 0 ) := ( others => '0' ); -- not clear is it worth using signed , technically we assume that within a window wont be change of the direction , strong assumption , I guess it may also fail
--signal pulse_counter : signed ( (log2 ( max_rpm_datasheet / 60) * 400 * window_period) downto 0 ) := ( others => '0' );
-- not log2 - 1 since if its 8 lets say it would give us 3 bits but instead we need 4 bits so 3 downto 0  , give it a check

begin

SYNC : process ( clk )
begin
	if rising_edge ( clk ) then
		a_sync1 <= channel_a;
		a_sync2 <= a_sync1;
		a_sync3 <= a_sync2;
	
		b_sync1 <= channel_b;
		b_sync2 <= b_sync1;
		b_sync3 <= b_sync2;
	end if;
end process SYNC;

FILTER_KIND_OF : process ( clk )
begin
	if rising_edge ( clk ) then
		a_ff1 <= a_sync_3;
		a_ff2 <= a_ff1;
		a_ff3 <= a_ff2;

		b_ff1 <= b_sync3
		b_ff2 <= b_ff1;
		b_ff3 <= b_ff2;
	end if;
end process FILTER_KIND_OF;

--=======COMB LOGIC TO determine if we had a stable input or not
state_a <= '1' when (a_ff1 and a_ff2 and a_ff3) or (not a_ff1 and not a_ff2 and not a_ff3) else
	   '0';

state_b <= '1' when ( b_ff1 and b_ff2 and b_ff3 ) or ( not b_ff1 and not b_ff2 and not b_ff3) else
	   '0';

FILTER_OUTPUT : process ( clk ) -- we could do it also a comb circuit
begin
	if rising_edge ( clk ) then
		if state_a then
			a_clean_curr <= a_ff3;
			a_clean_prev <= a_clean_curr;
			valid_a <= '1'; --better a fsm to be honest
		end if;
		if state_b then
			b_clean_curr <= b_ff3;
			b_clean_prev <= b_clean_Curr;
			valid_b <= '1' ; -- technically we expect the 2 channels to be stable at the same time 
		end if;
	end if;

end process FILTER_OUTPUT;




-- FSM for counting
type t_state is  ( FIRST , INCREMENT_1, INCREMENT_2 , INCREMENT_3 , INCREMENT_4 , DECREMENT_1 , DECREMENT_2, DECREMENT_3 , DECREMENT_4 , ERROR)
signal state : t_state := IDLE;

FSM : process ( clk )
begin
	if rising_edge (clk)
		case state is 
			when FIRST => 
				case ( a_clean_prev & a_clean_curr & b_clean_prev_ & b_clean_curr)
					when "0100" =>
						state <= INCREMENT_1;
					when "0001" => 
						state <= DECREMENT_1; 
					when "0101" or "1010" or "0110" or "1001" =>
						state <= ERROR ; --illegal states let us know of a problem to the interferences or noise or something
			when INCREMENT_1 =>
				state <= INCREMENT_1 ; --default
				cnt_stall <= 0;
				case ( a_clean_prev & a_clean_curr & b_clean_prev_ & b_clean_curr) 
					when "1101" =>
						state <= INCREMENT_2; -- even if it changes direction it would still go to 1101
						count <= count + 1;
					when "1100" =>
						if cnt_stall > STALL_THRESHOLD then
							state <= BRAKE;
						else
							cnt_stall <= cnt_stall + 1 ;
						end if;
					when others =>
						state <= error ;
				end case;
			when INCREMENT_2 =>
				state <= INCREMENT_2;
				cnt_stall <= (others =>'0');
				case ( a_clean_prev & a_clean_curr & b_clean_prev_ & b_clean_curr) 
					when "1011" =>
						state <= INCREMENT_3;
						count <= count + 1;
					when "1111" =>
						if cnt_stall > STALL_THRESHOLD then
							state <= BRAKE;
						else
							cnt_stall <= cnt_stall + 1 ;
						end if;
					when others =>
						state <= error;
				end case;
						
						
			when INCREMENT_3 =>
				state <= INCREMENT_3;
				cnt_stall <= ( others => '0');
				case ( a_clean_prev & a_clean_curr & b_clean_prev_ & b_clean_curr) 
					when "0010" =>
						state <= INCREMENT_4;
						count <= count + 1;-- for the previous state in fact
					when "0011" =>
						if cnt_stall > STALL_THRESHOLD then
							state <= BRAKE;
						else
							cnt_stall <= cnt_stall + 1 ;
						end if;
					when "0111" =>
						state <= FIRST;
						reset <= '1'; --reset the system and also the motor or just go to the appropriate state
					when others =>
						state <= error;
				end case;
			when INCREMENT_4 =>
				state <= INCREMENT_4;
				cnt_stall <= (others => '0');
				case ( a_clean_prev & a_clean_curr & b_clean_prev_ & b_clean_curr) 
					when "0100" =>
						state <= INCREMENT_1;
						count <= count + 1;-- for the previous state in fact
					when "0000" =>
						if cnt_stall > STALL_THRESHOLD then
							state <= BRAKE;
						else
							cnt_stall <= cnt_stall + 1 ;
						end if;
					when "0001" =>
						state <= FIRST;
						reset <= '1'; --reset the system and also the motor or just go to the appropriate state , because it looks that direction has changed
					when others =>
						state <= error;
				end case;
				
			when DECREMENT_1 =>
				state <= DECREMENT_1;
				cnt_stall <= ( others => '0');
				case ( a_clean_prev & a_clean_curr & b_clean_prev_ & b_clean_curr) 
					when "0111" =>
						state <= DECREMENT_2;
						count <= count - 1;-- for the next state in fact
					when "0011" =>
						if cnt_stall > STALL_THRESHOLD then
							state <= BRAKE;
						else
							cnt_stall <= cnt_stall + 1 ;
						end if;
					when others =>
						state <= error;
				end case;
			when DECREMENT_2 =>
				state <= DECREMENT_2;
				cnt_stall <= ( others => '0');
				case ( a_clean_prev & a_clean_curr & b_clean_prev_ & b_clean_curr) 
					when "1110" =>
						state <= DECREMENT_3;
						count <= count - 1;-- for the next state in fact
					when "1111" =>
						if cnt_stall > STALL_THRESHOLD then
							state <= BRAKE;
						else
							cnt_stall <= cnt_stall + 1 ;
						end if;
					when others =>
						state <= error;
				end case;
			when DECREMENT_3 =>
				state <= DECREMENT_3;
				cnt_stall <= ( others => '0');
				case ( a_clean_prev & a_clean_curr & b_clean_prev_ & b_clean_curr) 
					when "1000" =>
						state <= DECREMENT_4;
						count <= count - 1;-- for the next state in fact
					when "1100" =>
						if cnt_stall > STALL_THRESHOLD then
							state <= BRAKE;
						else
							cnt_stall <= cnt_stall + 1 ;
						end if;
					when others =>
						state <= error;
				end case;
			when DECREMENT_4 =>
				state <= DECREMENT_4;
				cnt_stall <= ( others => '0');
				case ( a_clean_prev & a_clean_curr & b_clean_prev_ & b_clean_curr) 
					when "0001" =>
						state <= DECREMENT_1;
						count <= count - 1;-- for the next state in fact
					when "0000" =>
						if cnt_stall > STALL_THRESHOLD then
							state <= BRAKE;
						else
							cnt_stall <= cnt_stall + 1 ;
						end if;
					when others =>
						state <= error;
				end case;

				
				
		if rising_edge ( clk ) then
		o_valid <= '0'; --default
		if cnt_timer = 49 then
			o_valid <= '1';
			count <= ( others => '0');
		end if;		

				
											

			 
	
end if;


end process FSM ; 

timer : process (clk)
begin
	if rising_edge ( clk )  then
		cnt_timer <= cnt + 1;

end process;

end architecture rtl of encoder

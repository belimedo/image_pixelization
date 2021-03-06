-- DESCRIPTION:
-- Input data will be 4 bits long:
-- 	-state 0000 means no LEDs should be active
-- 	-state 0001 means LED0 shoud be active
-- 	-state 0010 means LED1 shoud be active
-- 	-state 0100 means LED2 shoud be active
-- 	-state 1000 means LED3 shoud be active
-- Output is 10 bit std_logic_vector that represents LEDR vector 

library ieee;
use ieee.std_logic_1164.all;

-- Mislim da nam ne treba clk ovdje, tj. da ne treba biti sinhrono jer se izvrsava po promjeni data_input-a
entity display_LED_driver is
	port( 
		data_input		: in		std_logic_vector(3 downto 0);
		
		LEDR           : out   	std_logic_vector(9 downto 0)
	);
end entity display_LED_driver;

architecture arch of display_LED_driver is
begin
	LEDR(3 downto 0) <= data_input;
	LEDR(9 downto 4) <= "000000";
end architecture arch;
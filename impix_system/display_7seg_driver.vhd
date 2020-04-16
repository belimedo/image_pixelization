-- DESCRIPTION:
-- Input data will be 4 bits long:
-- 	-state 0000 means no displays should be active
-- 	-state 0001 means displays shoud show   2-2
-- 	-state 0010 means displays shoud show   4-4
-- 	-state 0100 means displays shoud show 	 8-8
-- 	-state 1000 means displays shoud show 16-16
-- Outputs are 7 bit std_logic_vectors that perform above given specification

library ieee;
use ieee.std_logic_1164.all;

-- Mislim da nam ne treba clk ovdje, tj. da ne treba biti sinhrono jer se izvrsava po promjeni data_input-a
entity display_7seg_driver is
	
	port( 
		data_input		: in		std_logic_vector(3 downto 0);
		
		HEX0           : out   	std_logic_vector(6 downto 0);
		HEX1           : out   	std_logic_vector(6 downto 0);
		HEX2           : out   	std_logic_vector(6 downto 0);
		HEX3           : out   	std_logic_vector(6 downto 0);
		HEX4           : out   	std_logic_vector(6 downto 0);
		HEX5           : out   	std_logic_vector(6 downto 0)
	);
end entity display_7seg_driver;

architecture arch of display_7seg_driver is
begin
	process(data_input)
	begin
		case data_input is
			when "0000" =>				--	Kada je ulaz 0000, iskljuci sve displeje
				HEX0 <= "1111111";
				HEX1 <= "1111111";
				HEX2 <= "1111111";
				HEX3 <= "1111111";
				HEX4 <= "1111111";
				HEX5 <= "1111111";
			when "0001" => 			-- Kada je ulaz 0001, ispisati 2-2
				HEX0 <= "0100100";
				HEX1 <= "0111111";
				HEX2 <= "0100100";
				HEX3 <= "1111111";
				HEX4 <= "1111111";
				HEX5 <= "1111111";
			when "0010" =>				-- Kada je ulaz 0010, ispisati 4-4
				HEX0 <= "0011001";
				HEX1 <= "0111111";
				HEX2 <= "0011001";
				HEX3 <= "1111111";
				HEX4 <= "1111111";
				HEX5 <= "1111111";
			when "0100" =>				-- Kada je ulaz 0100, ispisati 8-8
				HEX0 <= "0000000";
				HEX1 <= "0111111";
				HEX2 <= "0000000";
				HEX3 <= "1111111";
				HEX4 <= "1111111";
				HEX5 <= "1111111";
			when "1000" =>				-- Kada je ulaz 1000, ispisati 16-16
				HEX0 <= "0000010";
				HEX1 <= "1111001";
				HEX2 <= "0111111";
				HEX3 <= "0000010";
				HEX4 <= "1111001";
				HEX5 <= "1111111";
			when others =>
				null;
		end case;
	end process;
end architecture arch;
	
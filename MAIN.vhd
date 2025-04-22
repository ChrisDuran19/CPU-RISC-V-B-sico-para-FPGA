-- =================================================================
-- CPU RISC-V Básico para FPGA
-- Implementación simplificada del RV32I (RISC-V de 32 bits)
-- =================================================================

-- Paquete con definiciones comunes
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

package riscv_pkg is
    -- Definiciones de tipos comunes
    subtype word_t is std_logic_vector(31 downto 0);
    subtype reg_addr_t is std_logic_vector(4 downto 0);
    
    -- Códigos de operación RISC-V (opcode)
    constant OP_LUI    : std_logic_vector(6 downto 0) := "0110111";
    constant OP_AUIPC  : std_logic_vector(6 downto 0) := "0010111";
    constant OP_JAL    : std_logic_vector(6 downto 0) := "1101111";
    constant OP_JALR   : std_logic_vector(6 downto 0) := "1100111";
    constant OP_BRANCH : std_logic_vector(6 downto 0) := "1100011";
    constant OP_LOAD   : std_logic_vector(6 downto 0) := "0000011";
    constant OP_STORE  : std_logic_vector(6 downto 0) := "0100011";
    constant OP_IMM    : std_logic_vector(6 downto 0) := "0010011";
    constant OP_REG    : std_logic_vector(6 downto 0) := "0110011";
    
    -- Códigos de función ALU (funct3)
    constant F3_ADD_SUB : std_logic_vector(2 downto 0) := "000";
    constant F3_SLL    : std_logic_vector(2 downto 0) := "001";
    constant F3_SLT    : std_logic_vector(2 downto 0) := "010";
    constant F3_SLTU   : std_logic_vector(2 downto 0) := "011";
    constant F3_XOR    : std_logic_vector(2 downto 0) := "100";
    constant F3_SR     : std_logic_vector(2 downto 0) := "101";
    constant F3_OR     : std_logic_vector(2 downto 0) := "110";
    constant F3_AND    : std_logic_vector(2 downto 0) := "111";
    
    -- Códigos de función branch (funct3)
    constant F3_BEQ    : std_logic_vector(2 downto 0) := "000";
    constant F3_BNE    : std_logic_vector(2 downto 0) := "001";
    constant F3_BLT    : std_logic_vector(2 downto 0) := "100";
    constant F3_BGE    : std_logic_vector(2 downto 0) := "101";
    constant F3_BLTU   : std_logic_vector(2 downto 0) := "110";
    constant F3_BGEU   : std_logic_vector(2 downto 0) := "111";
    
    -- Códigos de función adicionales (funct7)
    constant F7_ADD    : std_logic_vector(6 downto 0) := "0000000";
    constant F7_SUB    : std_logic_vector(6 downto 0) := "0100000";
    constant F7_SRL    : std_logic_vector(6 downto 0) := "0000000";
    constant F7_SRA    : std_logic_vector(6 downto 0) := "0100000";
end package;

-- =================================================================
-- Banco de Registros (Register File)
-- =================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.riscv_pkg.all;

entity register_file is
    port (
        clk         : in std_logic;
        reset       : in std_logic;
        rs1_addr    : in reg_addr_t;  -- Dirección de registro fuente 1
        rs2_addr    : in reg_addr_t;  -- Dirección de registro fuente 2
        rd_addr     : in reg_addr_t;  -- Dirección de registro destino
        rd_data     : in word_t;      -- Datos a escribir
        write_en    : in std_logic;   -- Habilitación de escritura
        rs1_data    : out word_t;     -- Datos de registro fuente 1
        rs2_data    : out word_t      -- Datos de registro fuente 2
    );
end entity register_file;

architecture behavioral of register_file is
    -- 32 registros de 32 bits cada uno
    type reg_array_t is array (0 to 31) of word_t;
    signal registers : reg_array_t := (others => (others => '0'));
begin
    -- Lectura asíncrona (x0 es siempre 0)
    rs1_data <= (others => '0') when unsigned(rs1_addr) = 0 else registers(to_integer(unsigned(rs1_addr)));
    rs2_data <= (others => '0') when unsigned(rs2_addr) = 0 else registers(to_integer(unsigned(rs2_addr)));
    
    -- Escritura síncrona
    process(clk, reset)
    begin
        if reset = '1' then
            for i in 0 to 31 loop
                registers(i) <= (others => '0');
            end loop;
        elsif rising_edge(clk) then
            if write_en = '1' and unsigned(rd_addr) /= 0 then  -- No se permite escritura a x0
                registers(to_integer(unsigned(rd_addr))) <= rd_data;
            end if;
        end if;
    end process;
end architecture behavioral;

-- =================================================================
-- Unidad Aritmético-Lógica (ALU)
-- =================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.riscv_pkg.all;

entity alu is
    port (
        a           : in word_t;           -- Operando A
        b           : in word_t;           -- Operando B
        alu_op      : in std_logic_vector(3 downto 0);  -- Operación ALU
        result      : out word_t;          -- Resultado
        zero        : out std_logic;       -- Flag de cero
        less_than   : out std_logic        -- Flag de menor que
    );
end entity alu;

architecture behavioral of alu is
    signal result_temp : word_t;
    signal signed_less : std_logic;
    signal unsigned_less : std_logic;
begin
    process(a, b, alu_op)
        variable a_signed, b_signed : signed(31 downto 0);
        variable a_unsigned, b_unsigned : unsigned(31 downto 0);
    begin
        a_signed := signed(a);
        b_signed := signed(b);
        a_unsigned := unsigned(a);
        b_unsigned := unsigned(b);
        
        case alu_op is
            when "0000" =>  -- ADD
                result_temp <= std_logic_vector(a_unsigned + b_unsigned);
            when "0001" =>  -- SUB
                result_temp <= std_logic_vector(a_unsigned - b_unsigned);
            when "0010" =>  -- SLL: Shift Left Logical
                result_temp <= std_logic_vector(shift_left(a_unsigned, to_integer(b_unsigned(4 downto 0))));
            when "0011" =>  -- SLT: Set Less Than (signed)
                if a_signed < b_signed then
                    result_temp <= X"00000001";
                else
                    result_temp <= X"00000000";
                end if;
            when "0100" =>  -- SLTU: Set Less Than Unsigned
                if a_unsigned < b_unsigned then
                    result_temp <= X"00000001";
                else
                    result_temp <= X"00000000";
                end if;
            when "0101" =>  -- XOR
                result_temp <= a xor b;
            when "0110" =>  -- SRL: Shift Right Logical
                result_temp <= std_logic_vector(shift_right(a_unsigned, to_integer(b_unsigned(4 downto 0))));
            when "0111" =>  -- SRA: Shift Right Arithmetic
                result_temp <= std_logic_vector(shift_right(a_signed, to_integer(b_unsigned(4 downto 0))));
            when "1000" =>  -- OR
                result_temp <= a or b;
            when "1001" =>  -- AND
                result_temp <= a and b;
            when others =>
                result_temp <= (others => '0');
        end case;
    end process;
    
    result <= result_temp;
    zero <= '1' when unsigned(result_temp) = 0 else '0';
    signed_less <= '1' when signed(a) < signed(b) else '0';
    unsigned_less <= '1' when unsigned(a) < unsigned(b) else '0';
    less_than <= signed_less;
end architecture behavioral;

-- =================================================================
-- Unidad de Control
-- =================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.riscv_pkg.all;

entity control_unit is
    port (
        -- Señales de entrada
        opcode      : in std_logic_vector(6 downto 0);  -- Campo opcode de la instrucción
        funct3      : in std_logic_vector(2 downto 0);  -- Campo funct3 de la instrucción
        funct7      : in std_logic_vector(6 downto 0);  -- Campo funct7 de la instrucción
        zero        : in std_logic;                     -- Flag de cero de la ALU
        less_than   : in std_logic;                     -- Flag de menor que de la ALU
        
        -- Señales de control
        reg_write   : out std_logic;                   -- Habilitar escritura de registro
        alu_src     : out std_logic;                   -- Selección de fuente B de la ALU (0: rs2, 1: immediato)
        mem_write   : out std_logic;                   -- Habilitar escritura en memoria
        mem_read    : out std_logic;                   -- Habilitar lectura de memoria
        mem_to_reg  : out std_logic;                   -- Selección de fuente para registro (0: ALU, 1: memoria)
        branch      : out std_logic;                   -- Instrucción es un salto condicional
        jump        : out std_logic;                   -- Instrucción es un salto incondicional
        alu_op      : out std_logic_vector(3 downto 0); -- Operación ALU
        imm_sel     : out std_logic_vector(2 downto 0)  -- Selección del tipo de inmediato
    );
end entity control_unit;

architecture behavioral of control_unit is
    -- Tipos de formato de instrucción (para selección de inmediato)
    constant IMM_I : std_logic_vector(2 downto 0) := "000";  -- Inmediato tipo I
    constant IMM_S : std_logic_vector(2 downto 0) := "001";  -- Inmediato tipo S
    constant IMM_B : std_logic_vector(2 downto 0) := "010";  -- Inmediato tipo B
    constant IMM_U : std_logic_vector(2 downto 0) := "011";  -- Inmediato tipo U
    constant IMM_J : std_logic_vector(2 downto 0) := "100";  -- Inmediato tipo J
begin
    process(opcode, funct3, funct7, zero, less_than)
    begin
        -- Valores por defecto
        reg_write <= '0';
        alu_src <= '0';
        mem_write <= '0';
        mem_read <= '0';
        mem_to_reg <= '0';
        branch <= '0';
        jump <= '0';
        alu_op <= "0000";  -- ADD por defecto
        imm_sel <= IMM_I;  -- Tipo I por defecto
        
        case opcode is
            when OP_LUI =>     -- Load Upper Immediate
                reg_write <= '1';
                alu_src <= '1';
                alu_op <= "0000";  -- Simplemente pasa el inmediato
                imm_sel <= IMM_U;
                
            when OP_AUIPC =>   -- Add Upper Immediate to PC
                reg_write <= '1';
                alu_src <= '1';
                alu_op <= "0000";  -- ADD
                imm_sel <= IMM_U;
                
            when OP_JAL =>     -- Jump and Link
                reg_write <= '1';
                jump <= '1';
                imm_sel <= IMM_J;
                
            when OP_JALR =>    -- Jump and Link Register
                reg_write <= '1';
                alu_src <= '1';
                jump <= '1';
                imm_sel <= IMM_I;
                
            when OP_BRANCH =>  -- Branch
                branch <= '1';
                imm_sel <= IMM_B;
                alu_op <= "0001";  -- Resta para comparación
                
            when OP_LOAD =>    -- Load
                reg_write <= '1';
                mem_read <= '1';
                mem_to_reg <= '1';
                alu_src <= '1';
                alu_op <= "0000";  -- ADD para dirección
                imm_sel <= IMM_I;
                
            when OP_STORE =>   -- Store
                mem_write <= '1';
                alu_src <= '1';
                alu_op <= "0000";  -- ADD para dirección
                imm_sel <= IMM_S;
                
            when OP_IMM =>     -- Operaciones con inmediato
                reg_write <= '1';
                alu_src <= '1';
                imm_sel <= IMM_I;
                
                -- Seleccionar operación ALU basada en funct3
                case funct3 is
                    when F3_ADD_SUB => alu_op <= "0000";  -- ADD
                    when F3_SLL =>    alu_op <= "0010";  -- SLL
                    when F3_SLT =>    alu_op <= "0011";  -- SLT
                    when F3_SLTU =>   alu_op <= "0100";  -- SLTU
                    when F3_XOR =>    alu_op <= "0101";  -- XOR
                    when F3_SR =>     -- SRL o SRA
                        if funct7 = F7_SRL then
                            alu_op <= "0110";  -- SRL
                        else
                            alu_op <= "0111";  -- SRA
                        end if;
                    when F3_OR =>     alu_op <= "1000";  -- OR
                    when F3_AND =>    alu_op <= "1001";  -- AND
                    when others =>    alu_op <= "0000";  -- ADD por defecto
                end case;
                
            when OP_REG =>     -- Operaciones registro-registro
                reg_write <= '1';
                alu_src <= '0';
                
                -- Seleccionar operación ALU basada en funct3 y funct7
                case funct3 is
                    when F3_ADD_SUB =>  -- ADD o SUB
                        if funct7 = F7_SUB then
                            alu_op <= "0001";  -- SUB
                        else
                            alu_op <= "0000";  -- ADD
                        end if;
                    when F3_SLL =>    alu_op <= "0010";  -- SLL
                    when F3_SLT =>    alu_op <= "0011";  -- SLT
                    when F3_SLTU =>   alu_op <= "0100";  -- SLTU
                    when F3_XOR =>    alu_op <= "0101";  -- XOR
                    when F3_SR =>     -- SRL o SRA
                        if funct7 = F7_SRA then
                            alu_op <= "0111";  -- SRA
                        else
                            alu_op <= "0110";  -- SRL
                        end if;
                    when F3_OR =>     alu_op <= "1000";  -- OR
                    when F3_AND =>    alu_op <= "1001";  -- AND
                    when others =>    alu_op <= "0000";  -- ADD por defecto
                end case;
                
            when others =>     -- Instrucción no reconocida o no implementada
                null;  -- Mantiene valores por defecto
        end case;
    end process;
end architecture behavioral;

-- =================================================================
-- Generador de Inmediatos
-- =================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.riscv_pkg.all;

entity immediate_gen is
    port (
        instruction : in word_t;                      -- Instrucción completa
        imm_sel     : in std_logic_vector(2 downto 0); -- Selección de tipo de inmediato
        immediate   : out word_t                       -- Valor inmediato extendido
    );
end entity immediate_gen;

architecture behavioral of immediate_gen is
    -- Constantes para seleccionar tipo de inmediato
    constant IMM_I : std_logic_vector(2 downto 0) := "000";  -- Inmediato tipo I
    constant IMM_S : std_logic_vector(2 downto 0) := "001";  -- Inmediato tipo S
    constant IMM_B : std_logic_vector(2 downto 0) := "010";  -- Inmediato tipo B
    constant IMM_U : std_logic_vector(2 downto 0) := "011";  -- Inmediato tipo U
    constant IMM_J : std_logic_vector(2 downto 0) := "100";  -- Inmediato tipo J
begin
    process(instruction, imm_sel)
        variable imm_i : std_logic_vector(31 downto 0);
        variable imm_s : std_logic_vector(31 downto 0);
        variable imm_b : std_logic_vector(31 downto 0);
        variable imm_u : std_logic_vector(31 downto 0);
        variable imm_j : std_logic_vector(31 downto 0);
    begin
        -- Tipo I: Inmediato de 12 bits en [31:20]
        imm_i := (31 downto 12 => instruction(31)) & instruction(31 downto 20);
        
        -- Tipo S: Inmediato de 12 bits en [31:25] y [11:7]
        imm_s := (31 downto 12 => instruction(31)) & instruction(31 downto 25) & instruction(11 downto 7);
        
        -- Tipo B: Inmediato de 13 bits en [31], [7], [30:25], [11:8], '0'
        imm_b := (31 downto 13 => instruction(31)) & instruction(31) & instruction(7) & 
                instruction(30 downto 25) & instruction(11 downto 8) & '0';
        
        -- Tipo U: Inmediato de 20 bits en [31:12], seguido de 12 ceros
        imm_u := instruction(31 downto 12) & (11 downto 0 => '0');
        
        -- Tipo J: Inmediato de 21 bits en [31], [19:12], [20], [30:21], '0'
        imm_j := (31 downto 21 => instruction(31)) & instruction(31) & instruction(19 downto 12) & 
                instruction(20) & instruction(30 downto 21) & '0';
        
        -- Selección basada en el tipo de inmediato
        case imm_sel is
            when IMM_I => immediate <= imm_i;
            when IMM_S => immediate <= imm_s;
            when IMM_B => immediate <= imm_b;
            when IMM_U => immediate <= imm_u;
            when IMM_J => immediate <= imm_j;
            when others => immediate <= (others => '0');
        end case;
    end process;
end architecture behavioral;

-- =================================================================
-- Memoria de Programa
-- =================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.riscv_pkg.all;

entity program_memory is
    port (
        clk         : in std_logic;
        addr        : in word_t;               -- Dirección de memoria
        data_out    : out word_t               -- Dato leído
    );
end entity program_memory;

architecture behavioral of program_memory is
    -- Memoria ROM para las instrucciones (4KB)
    type rom_type is array (0 to 1023) of word_t;
    signal rom : rom_type := (
        -- Programa de ejemplo que suma los números del 1 al 10
        -- x1: contador, x2: suma, x3: límite
        0 => X"00100093",  -- addi x1, x0, 1      # x1 = 1 (contador)
        1 => X"00000113",  -- addi x2, x0, 0      # x2 = 0 (suma)
        2 => X"00A00193",  -- addi x3, x0, 10     # x3 = 10 (límite)
        3 => X"0030A663",  -- blt  x1, x3, 12     # si x1 < x3, saltar a la dirección PC+12 bytes (instrucción en posición 6)
        4 => X"00210113",  -- addi x2, x2, 2      # x2 = x2 + 2 (solo para prueba)
        5 => X"00000063",  -- beq  x0, x0, 0      # saltar a la dirección PC+0 bytes (bucle infinito)
        6 => X"002081B3",  -- add  x3, x1, x2     # x3 = x1 + x2
        7 => X"00118193",  -- addi x3, x3, 1      # x3 = x3 + 1
        8 => X"00000013",  -- nop
        others => (others => '0')
    );
begin
    -- Lectura asíncrona (combinacional)
    data_out <= rom(to_integer(unsigned(addr(11 downto 2))));  -- Alineado a palabras
end architecture behavioral;

-- =================================================================
-- Memoria de Datos
-- =================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.riscv_pkg.all;

entity data_memory is
    port (
        clk         : in std_logic;
        addr        : in word_t;               -- Dirección de memoria
        write_data  : in word_t;               -- Dato a escribir
        read_en     : in std_logic;            -- Habilitación de lectura
        write_en    : in std_logic;            -- Habilitación de escritura
        data_out    : out word_t               -- Dato leído
    );
end entity data_memory;

architecture behavioral of data_memory is
    -- Memoria RAM para los datos (4KB)
    type ram_type is array (0 to 1023) of word_t;
    signal ram : ram_type := (others => (others => '0'));
begin
    -- Proceso de escritura (síncrono)
    process(clk)
    begin
        if rising_edge(clk) then
            if write_en = '1' then
                ram(to_integer(unsigned(addr(11 downto 2)))) <= write_data;
            end if;
        end if;
    end process;
    
    -- Lectura asíncrona (combinacional)
    data_out <= ram(to_integer(unsigned(addr(11 downto 2)))) when read_en = '1' else (others => '0');
end architecture behavioral;

-- =================================================================
-- CPU RISC-V (Top Level)
-- =================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.riscv_pkg.all;

entity riscv_cpu is
    port (
        clk         : in std_logic;
        reset       : in std_logic;
        
        -- Para depuración
        dbg_pc      : out word_t;
        dbg_instruction : out word_t;
        dbg_reg_x1  : out word_t;
        dbg_reg_x2  : out word_t
    );
end entity riscv_cpu;

architecture behavioral of riscv_cpu is
    -- Señales para el PC
    signal pc_current    : word_t := (others => '0');
    signal pc_next       : word_t;
    signal pc_plus4      : word_t;
    signal pc_jump       : word_t;
    signal pc_branch     : word_t;
    
    -- Señales para instrucciones
    signal instruction   : word_t;
    signal opcode        : std_logic_vector(6 downto 0);
    signal rd_addr       : reg_addr_t;
    signal rs1_addr      : reg_addr_t;
    signal rs2_addr      : reg_addr_t;
    signal funct3        : std_logic_vector(2 downto 0);
    signal funct7        : std_logic_vector(6 downto 0);
    
    -- Señales de control
    signal reg_write     : std_logic;
    signal alu_src       : std_logic;
    signal mem_write     : std_logic;
    signal mem_read      : std_logic;
    signal mem_to_reg    : std_logic;
    signal branch        : std_logic;
    signal jump          : std_logic;
    signal alu_op        : std_logic_vector(3 downto 0);
    signal imm_sel       : std_logic_vector(2 downto 0);
    
    -- Señales para registros
    signal rs1_data      : word_t;
    signal rs2_data      : word_t;
    signal rd_data       : word_t;
    
    -- Señales para la ALU
    signal alu_a         : word_t;
    signal alu_b         : word_t;
    signal alu_result    : word_t;
    signal zero          : std_logic;
    signal less_than     : std_logic;
    
    -- Señales para memoria de datos
    signal mem_data      : word_t;
    
    -- Señales para inmediatos
    signal immediate     : word_t;
    
    -- Señal para tomar branch
    signal take_branch   : std_logic;
    
    -- Señales para debug
    signal reg_x1, reg_x2 : word_t;
begin
    -- Extracción de campos de la instrucción
    opcode <= instruction(6 downto 0);
    rd_addr <= instruction(11 downto 7);
    funct3 <= instruction(14 downto 12);
    rs1_addr <= instruction(19 downto 15);
    rs2_addr <= instruction(24 downto 20);
    funct7 <= instruction(31 downto 25);
    
    -- Instancia de memoria de programa
    prog_mem: entity work.program_memory
        port map (
            clk => clk,
            addr => pc_current,
            data_out => instruction
        );
    
    -- Instancia de unidad de control
    ctrl_unit: entity work.control_unit
        port map (
            opcode => opcode,
            funct3 => funct3,
            funct7 => funct7,
            zero => zero,
            less_than => less_than,
            reg_write => reg_write,
            alu_src => alu_src,
            mem_write => mem_write,
            mem_read => mem_read,
            mem_to_reg => mem_to_reg,
            branch => branch,
            jump => jump,
            alu_op => alu_op,
            imm_sel => imm_sel
        );
    
    -- Instancia de generador de inmediatos
    imm_gen: entity work.immediate_gen
        port map (
            instruction => instruction,
            imm_sel => imm_sel,
            immediate => immediate
        );
    
    -- Instancia de banco de registros
    reg_file: entity work.register_file
        port map (
            clk => clk,
            reset => reset,
            rs1_addr => rs1_addr,
            rs2_addr => rs2_addr,
            rd_addr => rd_addr,
            rd_data => rd_data,
            write_en => reg_write,
            rs1_data => rs1_data,
            rs2_data => rs2_data
        );
    
    -- Instancia de ALU
    alu_unit: entity work.alu
        port map (
            a => rs1_data,
            b => alu_b,
            alu_op => alu_op,
            result => alu_result,
            zero => zero,
            less_than => less_than
        );
    
    -- Instancia de memoria de datos
    data_mem: entity work.data_memory
        port map (
            clk => clk,
            addr => alu_result,
            write_data => rs2_data,
            read_en => mem_read,
            write_en => mem_write,
            data_out => mem_data
        );
    
    -- Lógica para tomar branch
    process(branch, funct3, zero, less_than)
    begin
        take_branch <= '0';
        
        if branch = '1' then
            case funct3 is
                when F3_BEQ =>  -- BEQ
                    take_branch <= zero;
                when F3_BNE =>  -- BNE
                    take_branch <= not zero;
                when F3_BLT =>  -- BLT
                    take_branch <= less_than;
                when F3_BGE =>  -- BGE
                    take_branch <= not less_than;
                when F3_BLTU => -- BLTU
                    take_branch <= less_than;
                when F3_BGEU => -- BGEU
                    take_branch <= not less_than;
                when others =>
                    take_branch <= '0';
            end case;
        end if;
    end process;
    
    -- Cálculo de próximo PC
    pc_plus4 <= std_logic_vector(unsigned(pc_current) + 4);
    pc_branch <= std_logic_vector(unsigned(pc_current) + unsigned(immediate));
    pc_jump <= std_logic_vector(unsigned(rs1_data) + unsigned(immediate)) when opcode = OP_JALR else
               std_logic_vector(unsigned(pc_current) + unsigned(immediate));
    
    -- Selector de próximo PC
    pc_next <= pc_jump when jump = '1' else
               pc_branch when (branch = '1' and take_branch = '1') else
               pc_plus4;
    
    -- Registro de PC
    process(clk, reset)
    begin
        if reset = '1' then
            pc_current <= (others => '0');
        elsif rising_edge(clk) then
            pc_current <= pc_next;
        end if;
    end process;
    
    -- Selector de fuente para operando B de la ALU
    alu_b <= immediate when alu_src = '1' else rs2_data;
    
    -- Selector de dato a escribir en registro
    rd_data <= pc_plus4 when (opcode = OP_JAL or opcode = OP_JALR) else
               mem_data when mem_to_reg = '1' else
               alu_result;
    
    -- Señales de depuración
    dbg_pc <= pc_current;
    dbg_instruction <= instruction;
    
    -- Para monitoreo de registros específicos
    process(clk)
    begin
        if rising_edge(clk) then
            if reg_write = '1' then
                if rd_addr = "00001" then  -- x1
                    reg_x1 <= rd_data;
                end if;
                if rd_addr = "00010" then  -- x2
                    reg_x2 <= rd_data;
                end if;
            end if;
        end if;
    end process;
    
    dbg_reg_x1 <= reg_x1;
    dbg_reg_x2 <= reg_x2;
end architecture behavioral;

-- =================================================================
-- Testbench para el CPU RISC-V
-- =================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.riscv_pkg.all;

entity riscv_cpu_tb is
end entity riscv_cpu_tb;

architecture testbench of riscv_cpu_tb is
    signal clk : std_logic := '0';
    signal reset : std_logic := '1';
    signal dbg_pc : word_t;
    signal dbg_instruction : word_t;
    signal dbg_reg_x1 : word_t;
    signal dbg_reg_x2 : word_t;
    
    constant CLK_PERIOD : time := 10 ns;
begin
    -- Instancia del DUT (Device Under Test)
    dut: entity work.riscv_cpu
        port map (
            clk => clk,
            reset => reset,
            dbg_pc => dbg_pc,
            dbg_instruction => dbg_instruction,
            dbg_reg_x1 => dbg_reg_x1,
            dbg_reg_x2 => dbg_reg_x2
        );
    
    -- Generación de reloj
    clk_gen: process
    begin
        clk <= '0';
        wait for CLK_PERIOD/2;
        clk <= '1';
        wait for CLK_PERIOD/2;
    end process;
    
    -- Proceso de estímulo
    stimulus: process
    begin
        -- Reset inicial
        reset <= '1';
        wait for CLK_PERIOD * 2;
        reset <= '0';
        
        -- Dejar que el CPU ejecute
        wait for CLK_PERIOD * 100;
        
        -- Finalizar simulación
        assert false report "Simulación completada" severity note;
        wait;
    end process;
    
    -- Proceso de monitoreo
    monitor: process
    begin
        wait for CLK_PERIOD;
        while true loop
            wait for CLK_PERIOD;
            report "PC: " & to_hstring(dbg_pc) & 
                   ", Instruction: " & to_hstring(dbg_instruction) &
                   ", x1 = " & to_hstring(dbg_reg_x1) &
                   ", x2 = " & to_hstring(dbg_reg_x2);
        end loop;
    end process;
end architecture testbench;

-- =================================================================
-- FPGA Top Level (para implementación física)
-- =================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.riscv_pkg.all;

entity fpga_top is
    port (
        clk_50mhz   : in std_logic;         -- Reloj de la FPGA (50 MHz típico)
        reset_n     : in std_logic;         -- Reset activo bajo (botón)
        led_out     : out std_logic_vector(7 downto 0)  -- LEDs para depuración
    );
end entity fpga_top;

architecture structural of fpga_top is
    signal reset : std_logic;
    signal clk : std_logic;
    signal clk_div : unsigned(23 downto 0) := (others => '0');
    signal dbg_pc, dbg_instruction, dbg_reg_x1, dbg_reg_x2 : word_t;
begin
    -- Invertir el reset activo bajo a activo alto
    reset <= not reset_n;
    
    -- Divisor de reloj (para visualización más lenta)
    process(clk_50mhz)
    begin
        if rising_edge(clk_50mhz) then
            clk_div <= clk_div + 1;
        end if;
    end process;
    
    -- Seleccionar velocidad de reloj
    clk <= clk_div(23);  -- Muy lento para depuración visual (aproximadamente 3 Hz)
    -- clk <= clk_50mhz;  -- Velocidad completa para operación normal
    
    -- Instancia del CPU RISC-V
    cpu: entity work.riscv_cpu
        port map (
            clk => clk,
            reset => reset,
            dbg_pc => dbg_pc,
            dbg_instruction => dbg_instruction,
            dbg_reg_x1 => dbg_reg_x1,
            dbg_reg_x2 => dbg_reg_x2
        );
    
    -- Conectar algunos bits de registros a LEDs para depuración
    led_out <= dbg_reg_x2(7 downto 0);  -- Mostrar los 8 bits menos significativos del registro x2 (suma)
end architecture structural;

define spi2_dump
   printf "CR1 = %04x\n",*(unsigned int*)(0x40003800)
   printf "CR2 = %04x\n",*(unsigned int*)(0x40003804)
   printf "SR  = %04x\n",*(unsigned int*)(0x40003808)
   if (*(unsigned int*)(0x40003800) & 0x0040)
      printf "SPE Set\n"
   else 
      printf "SPE Clear\n"
   end
end
document spi2_dump
Display the spi2 parameters 
Usage: spi2_dump
end

set disassemble-next-line on
target extended-remote :3333
load 
b main
c

.data
#out: .word 
0x400
#0xff 	#led, 初始全亮
#in: .word 
0xfe 		#switch

.text
#la a0, out		#仿真需要
lw t3, 0(x0)
sw x0, 8(t3) 		     #test sw: 全灭led
addi t0, x0, 0xff 	     #test addi: 全亮led
sw t0, 8(t3)
lw t0, 4(x0) 		     #test lw: 由switch设置led
sw t0, 8(t3)          #load in to out
addi t1,x0,1               #test add
add t0,t1,t0
sw t0, 8(t3)          #t0=t0+t1
sub t0,t0,t1          #t0=t0-t1
sw t0, 8(t3)          #load to out
addi t2,x0,1
auipc t1,0
loop1:
blt t2,x0,be
addi t2,t2,-1
sw t2, 8(t3)
jalr x1,t1,4                
be: addi t2,x0,1         
loop:
beq t2,x0,exit             #test beq
addi t2,t2,-1
sw t2, 8(t3)
jal x1,loop                #test jal
exit:


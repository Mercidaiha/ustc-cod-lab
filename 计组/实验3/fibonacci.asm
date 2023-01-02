.data
initial: .word 1,2
in:.word 4
out:.word 0
.text
la t0,initial
lw t1,0(t0) #t1=1
lw t2,4(t0) #t2=2
lw t4,8(t0)
addi t4,t4,-2
loop:
	beq t4,zero,exit
	add t3,t1,t2
	sw t3,12(t0)
	add t1,t2,zero
	add t2,t3,zero
	addi t4,t4,-1
jal x1 loop
exit:


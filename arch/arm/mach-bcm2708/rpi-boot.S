entry:
	mov r0, #0
	ldr r1, =3138		@ Machine ID
	ldr r2, =args
	ldr pc, =kern

.ltorg
table:

.balign 0x100, 0
args:
	.word 0x00000000	@ (length 0)
	.word 0x00000000	@ ATAG_NONE

.balign 0x8000, 0
kern:

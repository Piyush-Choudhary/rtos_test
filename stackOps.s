




;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------

   .def setPsp
   .def getPsp
   .def pushR4_11
   .def popR4_11


;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb
.const

;------------ -----------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------

.text

setPsp:

	MSR PSP,R0;
	MOV R0,#2;
	MSR CONTROL,R0; activate PSP
	ISB;
	BX LR;

getPsp:

	MRS R0,PSP
	BX LR


pushR4_11:

	MRS R0,PSP
	STMDB R0!, {R4-R11}
	MSR PSP, R0
	BX LR

popR4_11:

	MRS R0,PSP;
	LDMIA R0!,{R4-R11};
	MSR PSP,R0;
	BX LR;


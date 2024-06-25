################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../py/argcheck.c \
../py/asmarm.c \
../py/asmbase.c \
../py/asmthumb.c \
../py/asmx64.c \
../py/asmx86.c \
../py/asmxtensa.c \
../py/bc.c \
../py/binary.c \
../py/builtinevex.c \
../py/builtinhelp.c \
../py/builtinimport.c \
../py/compile.c \
../py/emitbc.c \
../py/emitcommon.c \
../py/emitglue.c \
../py/emitinlinethumb.c \
../py/emitinlinextensa.c \
../py/emitnarm.c \
../py/emitnx64.c \
../py/emitnx86.c \
../py/emitnxtensa.c \
../py/emitnxtensawin.c \
../py/formatfloat.c \
../py/frozenmod.c \
../py/gc.c \
../py/lexer.c \
../py/malloc.c \
../py/map.c \
../py/modarray.c \
../py/modbuiltins.c \
../py/modcmath.c \
../py/modcollections.c \
../py/moderrno.c \
../py/modgc.c \
../py/modio.c \
../py/modmath.c \
../py/modmicropython.c \
../py/modstruct.c \
../py/modsys.c \
../py/modthread.c \
../py/mpprint.c \
../py/mpstate.c \
../py/mpz.c \
../py/nativeglue.c \
../py/nlr.c \
../py/nlraarch64.c \
../py/nlrmips.c \
../py/nlrpowerpc.c \
../py/nlrrv32.c \
../py/nlrsetjmp.c \
../py/nlrthumb.c \
../py/nlrx64.c \
../py/nlrx86.c \
../py/nlrxtensa.c \
../py/obj.c \
../py/objarray.c \
../py/objattrtuple.c \
../py/objbool.c \
../py/objboundmeth.c \
../py/objcell.c \
../py/objclosure.c \
../py/objcomplex.c \
../py/objdeque.c \
../py/objdict.c \
../py/objenumerate.c \
../py/objexcept.c \
../py/objfilter.c \
../py/objfloat.c \
../py/objfun.c \
../py/objgenerator.c \
../py/objgetitemiter.c \
../py/objint.c \
../py/objint_longlong.c \
../py/objint_mpz.c \
../py/objlist.c \
../py/objmap.c \
../py/objmodule.c \
../py/objnamedtuple.c \
../py/objnone.c \
../py/objobject.c \
../py/objpolyiter.c \
../py/objproperty.c \
../py/objrange.c \
../py/objreversed.c \
../py/objset.c \
../py/objsingleton.c \
../py/objslice.c \
../py/objstr.c \
../py/objstringio.c \
../py/objstrunicode.c \
../py/objtuple.c \
../py/objtype.c \
../py/objzip.c \
../py/opmethods.c \
../py/pairheap.c \
../py/parse.c \
../py/parsenum.c \
../py/parsenumbase.c \
../py/persistentcode.c \
../py/profile.c \
../py/pystack.c \
../py/qstr.c \
../py/reader.c \
../py/repl.c \
../py/ringbuf.c \
../py/runtime.c \
../py/runtime_utils.c \
../py/scheduler.c \
../py/scope.c \
../py/sequence.c \
../py/showbc.c \
../py/smallint.c \
../py/stackctrl.c \
../py/stream.c \
../py/unicode.c \
../py/vm.c \
../py/vstr.c \
../py/warning.c 

C_DEPS += \
./py/argcheck.d \
./py/asmarm.d \
./py/asmbase.d \
./py/asmthumb.d \
./py/asmx64.d \
./py/asmx86.d \
./py/asmxtensa.d \
./py/bc.d \
./py/binary.d \
./py/builtinevex.d \
./py/builtinhelp.d \
./py/builtinimport.d \
./py/compile.d \
./py/emitbc.d \
./py/emitcommon.d \
./py/emitglue.d \
./py/emitinlinethumb.d \
./py/emitinlinextensa.d \
./py/emitnarm.d \
./py/emitnx64.d \
./py/emitnx86.d \
./py/emitnxtensa.d \
./py/emitnxtensawin.d \
./py/formatfloat.d \
./py/frozenmod.d \
./py/gc.d \
./py/lexer.d \
./py/malloc.d \
./py/map.d \
./py/modarray.d \
./py/modbuiltins.d \
./py/modcmath.d \
./py/modcollections.d \
./py/moderrno.d \
./py/modgc.d \
./py/modio.d \
./py/modmath.d \
./py/modmicropython.d \
./py/modstruct.d \
./py/modsys.d \
./py/modthread.d \
./py/mpprint.d \
./py/mpstate.d \
./py/mpz.d \
./py/nativeglue.d \
./py/nlr.d \
./py/nlraarch64.d \
./py/nlrmips.d \
./py/nlrpowerpc.d \
./py/nlrrv32.d \
./py/nlrsetjmp.d \
./py/nlrthumb.d \
./py/nlrx64.d \
./py/nlrx86.d \
./py/nlrxtensa.d \
./py/obj.d \
./py/objarray.d \
./py/objattrtuple.d \
./py/objbool.d \
./py/objboundmeth.d \
./py/objcell.d \
./py/objclosure.d \
./py/objcomplex.d \
./py/objdeque.d \
./py/objdict.d \
./py/objenumerate.d \
./py/objexcept.d \
./py/objfilter.d \
./py/objfloat.d \
./py/objfun.d \
./py/objgenerator.d \
./py/objgetitemiter.d \
./py/objint.d \
./py/objint_longlong.d \
./py/objint_mpz.d \
./py/objlist.d \
./py/objmap.d \
./py/objmodule.d \
./py/objnamedtuple.d \
./py/objnone.d \
./py/objobject.d \
./py/objpolyiter.d \
./py/objproperty.d \
./py/objrange.d \
./py/objreversed.d \
./py/objset.d \
./py/objsingleton.d \
./py/objslice.d \
./py/objstr.d \
./py/objstringio.d \
./py/objstrunicode.d \
./py/objtuple.d \
./py/objtype.d \
./py/objzip.d \
./py/opmethods.d \
./py/pairheap.d \
./py/parse.d \
./py/parsenum.d \
./py/parsenumbase.d \
./py/persistentcode.d \
./py/profile.d \
./py/pystack.d \
./py/qstr.d \
./py/reader.d \
./py/repl.d \
./py/ringbuf.d \
./py/runtime.d \
./py/runtime_utils.d \
./py/scheduler.d \
./py/scope.d \
./py/sequence.d \
./py/showbc.d \
./py/smallint.d \
./py/stackctrl.d \
./py/stream.d \
./py/unicode.d \
./py/vm.d \
./py/vstr.d \
./py/warning.d 

OBJS += \
./py/argcheck.o \
./py/asmarm.o \
./py/asmbase.o \
./py/asmthumb.o \
./py/asmx64.o \
./py/asmx86.o \
./py/asmxtensa.o \
./py/bc.o \
./py/binary.o \
./py/builtinevex.o \
./py/builtinhelp.o \
./py/builtinimport.o \
./py/compile.o \
./py/emitbc.o \
./py/emitcommon.o \
./py/emitglue.o \
./py/emitinlinethumb.o \
./py/emitinlinextensa.o \
./py/emitnarm.o \
./py/emitnx64.o \
./py/emitnx86.o \
./py/emitnxtensa.o \
./py/emitnxtensawin.o \
./py/formatfloat.o \
./py/frozenmod.o \
./py/gc.o \
./py/lexer.o \
./py/malloc.o \
./py/map.o \
./py/modarray.o \
./py/modbuiltins.o \
./py/modcmath.o \
./py/modcollections.o \
./py/moderrno.o \
./py/modgc.o \
./py/modio.o \
./py/modmath.o \
./py/modmicropython.o \
./py/modstruct.o \
./py/modsys.o \
./py/modthread.o \
./py/mpprint.o \
./py/mpstate.o \
./py/mpz.o \
./py/nativeglue.o \
./py/nlr.o \
./py/nlraarch64.o \
./py/nlrmips.o \
./py/nlrpowerpc.o \
./py/nlrrv32.o \
./py/nlrsetjmp.o \
./py/nlrthumb.o \
./py/nlrx64.o \
./py/nlrx86.o \
./py/nlrxtensa.o \
./py/obj.o \
./py/objarray.o \
./py/objattrtuple.o \
./py/objbool.o \
./py/objboundmeth.o \
./py/objcell.o \
./py/objclosure.o \
./py/objcomplex.o \
./py/objdeque.o \
./py/objdict.o \
./py/objenumerate.o \
./py/objexcept.o \
./py/objfilter.o \
./py/objfloat.o \
./py/objfun.o \
./py/objgenerator.o \
./py/objgetitemiter.o \
./py/objint.o \
./py/objint_longlong.o \
./py/objint_mpz.o \
./py/objlist.o \
./py/objmap.o \
./py/objmodule.o \
./py/objnamedtuple.o \
./py/objnone.o \
./py/objobject.o \
./py/objpolyiter.o \
./py/objproperty.o \
./py/objrange.o \
./py/objreversed.o \
./py/objset.o \
./py/objsingleton.o \
./py/objslice.o \
./py/objstr.o \
./py/objstringio.o \
./py/objstrunicode.o \
./py/objtuple.o \
./py/objtype.o \
./py/objzip.o \
./py/opmethods.o \
./py/pairheap.o \
./py/parse.o \
./py/parsenum.o \
./py/parsenumbase.o \
./py/persistentcode.o \
./py/profile.o \
./py/pystack.o \
./py/qstr.o \
./py/reader.o \
./py/repl.o \
./py/ringbuf.o \
./py/runtime.o \
./py/runtime_utils.o \
./py/scheduler.o \
./py/scope.o \
./py/sequence.o \
./py/showbc.o \
./py/smallint.o \
./py/stackctrl.o \
./py/stream.o \
./py/unicode.o \
./py/vm.o \
./py/vstr.o \
./py/warning.o 


# Each subdirectory must supply rules for building sources it contributes
py/%.o: ../py/%.c py/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU RISC-V Cross C Compiler'
	riscv64-zephyr-elf-gcc -march=rv32imc_zicsr -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -g3 -I/home/dinesha/workarea/github/micropython/ports/riscduino/bsp/inc -I/home/dinesha/workarea/github/micropython/py -I/home/dinesha/workarea/github/micropython/ports/riscduino/build -I/home/dinesha/workarea/github/micropython/shared/readline -I/home/dinesha/workarea/github/micropython/shared/runtime -I/home/dinesha/workarea/github/micropython -I/home/dinesha/workarea/github/micropython/ports/riscduino -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



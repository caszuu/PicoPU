import os
import llvmlite.binding as llvm

# init pipeline compiler

llvm.initialize()
llvm.initialize_all_targets()
llvm.initialize_all_asmprinters()

shader_target = llvm.Target.from_triple("arm-none-eabi")

# shader_machine = shader_target.create_target_machine(cpu="cortex-m0plus", opt=2, reloc="default", codemodel="jitdefault", jit=False, abiname="")
shader_machine = shader_target.create_target_machine(cpu="cortex-m33", opt=2, reloc="default", codemodel="jitdefault", jit=False, abiname="")

# emulated broker

llvm_ir = """
; ModuleID = 'test.c'
source_filename = "test.c"
target datalayout = "e-m:e-p:32:32-Fi8-i64:64-v128:64:128-a:0:32-n32-S64"
target triple = "thumbv8m.main-none-unknown-eabi"

@buf = dso_local local_unnamed_addr global [128 x i8] zeroinitializer, align 1

; Function Attrs: mustprogress nofree norecurse nosync nounwind optsize willreturn memory(argmem: write)
define dso_local void @frag_entry(ptr noalias nocapture sret(<8 x i32>) align 8 %0, <8 x i32> noundef %1, ptr nocapture noundef readnone %2, i32 noundef %3, i8 noundef zeroext %4) local_unnamed_addr #0 {
  %6 = mul <8 x i32> %1, <i32 5, i32 5, i32 5, i32 5, i32 5, i32 5, i32 5, i32 5>
  store <8 x i32> %6, ptr %0, align 8, !tbaa !4
  ret void
}

; Function Attrs: nounwind optsize
define dso_local void @test_stall() local_unnamed_addr #1 {
  %1 = load i8, ptr @buf, align 1, !tbaa !4
  %2 = add i8 %1, 1
  store i8 %2, ptr @buf, align 1, !tbaa !4
  tail call void @shader_stall() #3
  ret void
}

; Function Attrs: optsize
declare dso_local void @shader_stall(...) local_unnamed_addr #2

attributes #0 = { mustprogress nofree norecurse nosync nounwind optsize willreturn "frame-pointer"="all" "no-trapping-math"="true" "stack-protector-buffer-size"="8" "target-cpu"="cortex-m33" "target-features"="+armv8-m.main,+dsp,+fp-armv8d16sp,+fp16,+hwdiv,+strict-align,+thumb-mode,+vfp2sp,+vfp3d16sp,+vfp4d16sp,-aes,-bf16,-cdecp0,-cdecp1,-cdecp2,-cdecp3,-cdecp4,-cdecp5,-cdecp6,-cdecp7,-crc,-crypto,-d32,-dotprod,-fp-armv8,-fp-armv8d16,-fp-armv8sp,-fp16fml,-fp64,-fullfp16,-hwdiv-arm,-i8mm,-lob,-mve,-mve.fp,-neon,-pacbti,-ras,-sb,-sha2,-vfp2,-vfp3,-vfp3d16,-vfp3sp,-vfp4,-vfp4d16,-vfp4sp" }
attributes #1 = { nounwind optsize "frame-pointer"="all" "no-trapping-math"="true" "stack-protector-buffer-size"="8" "target-cpu"="cortex-m33" "target-features"="+armv8-m.main,+dsp,+fp-armv8d16sp,+fp16,+hwdiv,+strict-align,+thumb-mode,+vfp2sp,+vfp3d16sp,+vfp4d16sp,-aes,-bf16,-cdecp0,-cdecp1,-cdecp2,-cdecp3,-cdecp4,-cdecp5,-cdecp6,-cdecp7,-crc,-crypto,-d32,-dotprod,-fp-armv8,-fp-armv8d16,-fp-armv8sp,-fp16fml,-fp64,-fullfp16,-hwdiv-arm,-i8mm,-lob,-mve,-mve.fp,-neon,-pacbti,-ras,-sb,-sha2,-vfp2,-vfp3,-vfp3d16,-vfp3sp,-vfp4,-vfp4d16,-vfp4sp" }
attributes #2 = { optsize "frame-pointer"="all" "no-trapping-math"="true" "stack-protector-buffer-size"="8" "target-cpu"="cortex-m33" "target-features"="+armv8-m.main,+dsp,+fp-armv8d16sp,+fp16,+hwdiv,+strict-align,+thumb-mode,+vfp2sp,+vfp3d16sp,+vfp4d16sp,-aes,-bf16,-cdecp0,-cdecp1,-cdecp2,-cdecp3,-cdecp4,-cdecp5,-cdecp6,-cdecp7,-crc,-crypto,-d32,-dotprod,-fp-armv8,-fp-armv8d16,-fp-armv8sp,-fp16fml,-fp64,-fullfp16,-hwdiv-arm,-i8mm,-lob,-mve,-mve.fp,-neon,-pacbti,-ras,-sb,-sha2,-vfp2,-vfp3,-vfp3d16,-vfp3sp,-vfp4,-vfp4d16,-vfp4sp" }
attributes #3 = { nounwind optsize }

!llvm.module.flags = !{!0, !1, !2}
!llvm.ident = !{!3}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{i32 1, !"min_enum_size", i32 4}
!2 = !{i32 7, !"frame-pointer", i32 2}
!3 = !{!"clang version 18.1.8"}
!4 = !{!5, !5, i64 0}
!5 = !{!"omnipotent char", !6, i64 0}
!6 = !{!"Simple C/C++ TBAA"}
"""

ex_ir = """
; ModuleID = 'test.c'
source_filename = "test.c"
target datalayout = "e-m:e-p:32:32-Fi8-i64:64-v128:64:128-a:0:32-n32-S64"
target triple = "thumbv8m.main-none-unknown-eabi"

; Function Attrs: mustprogress nofree norecurse nosync nounwind optsize willreturn memory(argmem: readwrite)
define dso_local void @frag_entry(ptr nocapture noundef %0, ptr nocapture noundef readnone %1, i32 noundef %2, i8 noundef zeroext %3) local_unnamed_addr #0 {
  %5 = load <4 x i32>, ptr %0, align 4, !tbaa !4
  %6 = mul <4 x i32> %5, <i32 5, i32 5, i32 5, i32 5>
  store <4 x i32> %6, ptr %0, align 4, !tbaa !4
  ret void
}

attributes #0 = { mustprogress nofree norecurse nosync nounwind optsize willreturn "frame-pointer"="all" "no-trapping-math"="true" "stack-protector-buffer-size"="8" "target-cpu"="cortex-m33" "target-features"="+armv8-m.main,+dsp,+fp-armv8d16sp,+fp16,+hwdiv,+strict-align,+thumb-mode,+vfp2sp,+vfp3d16sp,+vfp4d16sp,-aes,-bf16,-cdecp0,-cdecp1,-cdecp2,-cdecp3,-cdecp4,-cdecp5,-cdecp6,-cdecp7,-crc,-crypto,-d32,-dotprod,-fp-armv8,-fp-armv8d16,-fp-armv8sp,-fp16fml,-fp64,-fullfp16,-hwdiv-arm,-i8mm,-lob,-mve,-mve.fp,-neon,-pacbti,-ras,-sb,-sha2,-vfp2,-vfp3,-vfp3d16,-vfp3sp,-vfp4,-vfp4d16,-vfp4sp" }

!llvm.module.flags = !{!0, !1, !2}
!llvm.ident = !{!3}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{i32 1, !"min_enum_size", i32 4}
!2 = !{i32 7, !"frame-pointer", i32 2}
!3 = !{!"clang version 18.1.8"}
!4 = !{!5, !5, i64 0}
!5 = !{!"omnipotent char", !6, i64 0}
!6 = !{!"Simple C/C++ TBAA"}

"""

# required to set before llvm 15
llvm.options.set_option("opq-ptr", "-opaque-pointers")

module = llvm.parse_assembly(llvm_ir)
print(shader_machine.emit_assembly(module))

with open('s.o', 'wb') as f:
  f.write(shader_machine.emit_object(module))

module.verify()

# link shader binary

ld_path = "/home/caszu/.pico-sdk/toolchain/13_2_Rel1/bin/arm-none-eabi-ld"
fware_path = "../core-firmware/build/shader_core/shader_firmware.elf"

os.system(ld_path + " --just-symbols " + fware_path + " -o s.bin s.o")

# shutdown

llvm.shutdown()

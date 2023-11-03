#SEQUENCE="_Fast1_ECID_decay5_down1_interp30_noicp"
SEQUENCE=""
PREPRECESS=1
APE=0
RTE=0
VISUAL=1


if [ $VISUAL -eq 0 ]; then
    VISUAL=""
else
    VISUAL="-p"
fi

if [ $PREPRECESS -eq 1 ]; then
    echo "===PROPROCESSING==="
    python3 ./preprocess.py
    echo "===PROPROCESSED==="
fi

#$"gt"$SEQUENCE$".txt"
GT_SEQ=$"gt"$SEQUENCE$".txt"
RES_SEQ=$"result"$SEQUENCE$".txt"

echo "===TWO SEQUENCE==="
echo $RES_SEQ
echo $GT_SEQ
echo "===TWO SEQUENCE==="

cd ok2compare

evo_traj tum --ref=$GT_SEQ $RES_SEQ 
echo "===+++@@@@@@+++==="
evo_rpe tum $GT_SEQ $RES_SEQ -a -r trans_part $VISUAL
evo_rpe tum $GT_SEQ $RES_SEQ -a -r angle_deg $VISUAL
echo "===+++@@@@@@+++==="
evo_ape tum $GT_SEQ $RES_SEQ -a -r trans_part $VISUAL
evo_ape tum $GT_SEQ $RES_SEQ -a -r angle_deg $VISUAL

cd ..

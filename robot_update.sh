#!/bin/bash

DOWNLOADS="/media/psf/Home/Downloads"
PROJECT="/home/jbf/Documents/Electronique/NIOS2/Robot_new"

QUARTUS_BIN="/home/jbf/intelFPGA_standard/24.1std/quartus/bin"
QUARTUS_SH="$QUARTUS_BIN/quartus_sh"
QUARTUS_PGM="$QUARTUS_BIN/quartus_pgm"
JTAGCONFIG="$QUARTUS_BIN/jtagconfig"

PROJECT_NAME="top"
FIT_DIR="$PROJECT/fit"
SOF="$FIT_DIR/output_files/top.sof"

LAST=""

echo "============================================"
echo " Robot automatic build + program"
echo "============================================"
echo "Downloads : $DOWNLOADS"
echo "Project   : $PROJECT"
echo

while true
do
    # Dernier dossier Robot_* téléchargé
    ROBOT_DIR=$(
        find "$DOWNLOADS" \
            -maxdepth 1 \
            -mindepth 1 \
            -type d \
            -name "Robot_*" \
            -print |
        while IFS= read -r DIR
        do
            NAME=$(basename "$DIR")

            VERSION=$(echo "$NAME" | sed -n 's/.*_v\([0-9][0-9]*\).*/\1/p')

            if [ -n "$VERSION" ]; then
                printf "%06d|%s\n" "$VERSION" "$DIR"
            fi
        done |
        sort -n |
        tail -1 |
        cut -d'|' -f2-
    )

    if [ -n "$ROBOT_DIR" ] && [ "$ROBOT_DIR" != "$LAST" ]; then

        if [ -d "$ROBOT_DIR/src" ] && [ -d "$ROBOT_DIR/fit" ]; then

            echo
            echo "=================================================="
            echo "NOUVELLE VERSION : $(basename "$ROBOT_DIR")"
            echo "=================================================="

            # -------------------------------------------------
            # 1. Copie des sources
            # -------------------------------------------------
            echo
            echo "[1/4] Copie src/"

            rsync -av \
                "$ROBOT_DIR/src/" \
                "$PROJECT/src/"

            if [ $? -ne 0 ]; then
                echo "ERREUR pendant copie src"
                LAST="$ROBOT_DIR"
                continue
            fi

            echo
            echo "[2/4] Copie fit/"

            rsync -av \
                "$ROBOT_DIR/fit/" \
                "$PROJECT/fit/"

            if [ $? -ne 0 ]; then
                echo "ERREUR pendant copie fit"
                LAST="$ROBOT_DIR"
                continue
            fi

            # -------------------------------------------------
            # 2. Compilation Quartus
            # -------------------------------------------------
            echo
            echo "=================================================="
            echo "[3/4] COMPILATION QUARTUS"
            echo "=================================================="

            cd "$FIT_DIR" || exit 1

            "$QUARTUS_SH" --flow compile "$PROJECT_NAME"

            COMPILE_RESULT=$?

            if [ $COMPILE_RESULT -ne 0 ]; then
                echo
                echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                echo "ERREUR DE COMPILATION"
                echo "FPGA NON PROGRAMME"
                echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"

                LAST="$ROBOT_DIR"
                continue
            fi

            # Vérification que le SOF existe
            if [ ! -f "$SOF" ]; then
                echo
                echo "ERREUR :"
                echo "$SOF introuvable"
                echo "FPGA NON PROGRAMME"

                LAST="$ROBOT_DIR"
                continue
            fi

            echo
            echo "Compilation OK"
            echo "SOF : $SOF"

            # -------------------------------------------------
            # 3. Vérification JTAG
            # -------------------------------------------------
            echo
            echo "Vérification USB-Blaster / MAX10..."

            "$JTAGCONFIG"

            if [ $? -ne 0 ]; then
                echo
                echo "ERREUR JTAG"
                echo "FPGA NON PROGRAMME"

                LAST="$ROBOT_DIR"
                continue
            fi

            # -------------------------------------------------
            # 4. Programmation FPGA
            # -------------------------------------------------
            echo
            echo "=================================================="
            echo "[4/4] PROGRAMMATION FPGA"
            echo "=================================================="

            "$QUARTUS_PGM" \
                -c 1 \
                -m JTAG \
                -o "p;$SOF@1"

            PROGRAM_RESULT=$?

            if [ $PROGRAM_RESULT -eq 0 ]; then
                echo
                echo "============================================"
                echo "✓ COMPILATION OK"
                echo "✓ FPGA PROGRAMME"
                echo "============================================"
            else
                echo
                echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                echo "ERREUR PROGRAMMATION FPGA"
                echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            fi

            LAST="$ROBOT_DIR"
        fi
    fi

    sleep 1
done
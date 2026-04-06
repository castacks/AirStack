#!/bin/bash

if [ ! -f model.dae ]; then
    gdown 1VX9-B0FgCXw2Fl8cQvn_efaTeBmbQ3Sz -O model.dae
fi

if [ ! -d fire_academy_decimate_half.fbm ]; then
    gdown 1ngHLovPWGgTFl-RQC-OfTNTN_Wju6j6l -O fire_academy_decimate_half.fbm.zip
    unzip fire_academy_decimate_half.fbm.zip -d fire_academy_decimate_half.fbm
    rm fire_academy_decimate_half.fbm.zip
fi

if [ ! -f fire_academy_no_box.fbx ]; then
    gdown 1GlouTPUkfmRFSAzXZ78tTDC6ZHZGUGVP -O fire_academy_no_box.fbx
fi


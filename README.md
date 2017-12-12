# AR_MMD
OpenCVとOpenGLで作ったARプログラム．

MMDを読み込むためのライブラリsaba( https://github.com/benikabocha/saba )とビューアのためのライブラリ( https://github.com/stevenlovegrove/Pangolin )を使っている．  

## ビルド  
`mkdir build && cd build`  
`cmake .. && make`

## 使い方  
`./mmd_at [target image(../0.jpg)] [calibration folder(../calibration_empty)]`


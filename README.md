# AR_MMD
OpenCVとOpenGLで作ったARプログラム．

MMDを読み込むためのライブラリsaba( https://github.com/benikabocha/saba )とビューアのためのライブラリ( https://github.com/stevenlovegrove/Pangolin )を使っている．  

## ビルド  
sabaとPangolinの依存はそれぞれのgithub参照．  
`bash ./build.sh`  

## 使い方  
`./mmd_ar [target image(../0.jpg)] [calibration folder(../calibration_empty)]`


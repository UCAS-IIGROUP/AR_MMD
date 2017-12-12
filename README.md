# AR_MMD
OpenCVとOpenGLで作ったARプログラム．

MMDを読み込むためのライブラリsaba( https://github.com/benikabocha/saba )とビューアのためのライブラリ( https://github.com/stevenlovegrove/Pangolin )を使っている．  

## ビルド  
sabaとPangolinの依存はそれぞれのgithub参照．
`cd third_party && rm -rf *`
`git clone git@github.com:benikabocha/saba.git`
`cd saba && mkdir build && cd build && cmake .. && make -j2`
`cd ../..`
`git clone git@github.com:stevenlovegrove/Pangolin.git`
`cd Pangolin && mkdir build && cd build && cmake .. && make -j2`
`mkdir build && cd build`  
`cmake .. && make`

## 使い方  
`./mmd_ar [target image(../0.jpg)] [calibration folder(../calibration_empty)]`


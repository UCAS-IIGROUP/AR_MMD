# AR_MMD
OpenCVとOpenGLで作ったARプログラム．

## ビルド  
#### 依存
Saba - https://github.com/benikabocha/saba  
Pangolin - https://github.com/stevenlovegrove/Pangolin  

sabaとPangolinの依存はそれぞれのgithub参照．  
`bash ./build.sh`  

## 使い方  
`./mmd_ar [target image(../0.jpg)] [calibration folder(../calibration_empty)]`

### 作りかけ  
1. ARUCOを使うモードを作成中．
2. 特徴点ベースは遅いので別な平面追跡アルゴリズム（画像の勾配情報を使うような）を検討中

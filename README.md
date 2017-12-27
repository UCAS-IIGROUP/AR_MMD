# AR_MMD
OpenCVとOpenGLで作ったARプログラム．

## ビルド  
`bash ./build.sh`  

#### 依存
Saba - https://github.com/benikabocha/saba  
Pangolin - https://github.com/stevenlovegrove/Pangolin  

## 使い方  
`./mmd_ar [target image(../target_example.jpg)] [calibration folder(../calibration_empty)]`  
`./mmd_ar [target image(../target_marker.jpg)] [calibration folder(../calibration_empty)] # マーカーモード` 
`./mmd_ar_simulator [target image(../target_example.jpg)] [calibration folder(../calibration_empty)] # シミュレータモード` 

### 作りかけ  
１. 特徴点ベースは遅いので別な平面追跡アルゴリズム（画像の勾配情報を使うような）を検討中

### 注意
mmd_ar_simulatorのターゲットにARマーカーを指定しても現在は動作しない．  

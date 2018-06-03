# AR_MMD
OpenCVとOpenGLで作ったMMD用ARプログラム．  

## ビルド  
OpenCV(>3.1)とOpenCV_contrib(>3.1)をインストールしてから以下のコマンド．  
`bash ./build.sh`  

#### 依存
OpenCV - https://github.com/opencv/opencv  
OpenCV_contrib - https://github.com/opencv/opencv_contrib  
Saba - https://github.com/benikabocha/saba  
Pangolin - https://github.com/stevenlovegrove/Pangolin  

## 使い方  
`./mmd_ar [target image(../target_example.jpg)] [calibration folder(../calibration_empty)]`  
`./mmd_ar [target marker(../target_marker.jpg)] [calibration folder] # マーカーモード`  
`./mmd_ar_simulator [target image(../target_marker.jpg)] [calibration folder] # シミュレータモード`  

#### キャリブレーション
`d_param.txt`には歪みパラメータを，`k_param.txt`にはカメラの内部パラメータを入れておく．`param.txt`にはARシステムに与える入力画像の横幅と高さを入れておく．形式はリポジトリの`calibration_empty`フォルダにある各ファイルを参照．


### おまけ  
カメラキャプチャ   
`python cap.py`  
カメラキャリブレーション  
`python camera_calibration.py`  

### 作りかけ  
1. 特徴点ベースは遅いので別な平面追跡アルゴリズム（画像の勾配情報を使うような）を検討中  
2. MMDモデル描画の改善．  

### 注意
mmd_ar_simulatorのターゲットにARマーカーを指定しても現在は動作しない．  

### Demo
https://www.youtube.com/watch?v=UmBR0Bx28Po

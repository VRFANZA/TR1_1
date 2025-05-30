#include <Novice.h>
#include <sstream>
#include <vector>
#include "SerialReader.h"

const char kWindowTitle[] = "LC1C_14_タカムラシュン_タイトル";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	// グローバル or 上部で宣言
	SerialReader* serial = nullptr;
	std::vector<int> sensorData(9);

	// WinMain の初期化部分
	serial = new SerialReader("COM6");  // ArduinoのCOMポート

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		// 毎フレームの更新処理
		std::string line;
		if (serial->readLine(line)) {
			std::istringstream ss(line);
			std::string token;
			for (int i = 0; std::getline(ss, token, ',') && i < 9; ++i) {
				sensorData[i] = std::stoi(token);
			}
		}

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		// Headingの値を画面に描画する（左上に表示）
		Novice::ScreenPrintf(10, 10, "Heading: %d", sensorData[0]);

		// PitchとRollも一緒に表示
		Novice::ScreenPrintf(10, 30, "Pitch: %d", sensorData[1]);
		Novice::ScreenPrintf(10, 50, "Roll:  %d", sensorData[2]);

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}

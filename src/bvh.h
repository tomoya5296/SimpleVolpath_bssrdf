#ifndef _BVH_H_
#define _BVH_H_
#include <vector>
#include "TRIANGLE.h"
#include <iostream>
#include <algorithm>
// BVH のノードの構造
struct BVH_node {
	float bbox[2][3];  // bbox[0,1]: AABB の最小,最大座標． bbox[hoge][0,1,2]: x,y,z座標
	int children[2];    // 子ノード
	std::vector<TRIANGLE *> polygons;  // 格納されているポリゴン (葉ノードのみ有効)
};
// AABB の表面積計算
float surfaceArea(const float bbox[2][3]) {
	float dx = bbox[1][0] - bbox[0][0];
	float dy = bbox[1][1] - bbox[0][1];
	float dz = bbox[1][2] - bbox[0][2];
	return 2 * ((dx*dy) + (dx*dz) + (dy*dz));
}
// 空の AABB を作成
void  emptyAABB (float bbox[2][3]) {
	bbox[0][0] = bbox[0][1] = bbox[0][2] = INF;
	bbox[1][0] = bbox[1][1] = bbox[1][2] = -INF;
}
// 2つのAABBをマージ
void mergeAABB(const float bbox1[2][3], const float bbox2[2][3], float result[2][3]) {
	for (int j = 0; j<3; j++) {
		result[0][j] =std::min(bbox1[0][j], bbox2[0][j]);
		result[1][j] =std::max(bbox1[1][j], bbox2[1][j]);
	}
}
// ポリゴンリストからAABBを生成
void creatAABBfromTriangles(const std::vector<TRIANGLE *> &triangles, float bbox[2][3]) {
	emptyAABB(bbox);

	std::for_each(triangles.begin(), triangles.end(), [&bbox](const TRIANGLE *t) {
		mergeAABB(t->bbox, bbox, bbox);
	});
}


BVH_node nodes[10000];  // ノードリスト．本当は適切なノード数を確保すべき
int used_node_count = 0;  // 現在使用されているノードの数
float T_tri = 1.0;  // 適当				
float T_aabb = 1.0;  // 適当			

// nodeIndex で指定されたノードを、polygons を含む葉ノードにする
void makeLeaf(std::vector<TRIANGLE *> &polygons, BVH_node *node) {
	node->children[0] = node->children[1] = -1;
	node->polygons = polygons;
}


// 与えられたポリゴンリストについて、SAH に基づいて領域分割
// nodeIndex は対象ノードのインデックス
// 
void constructBVH_internal(std::vector<TRIANGLE *> &polygons, int nodeIndex) {
	BVH_node *node = &nodes[nodeIndex];
	
	creatAABBfromTriangles(polygons, node->bbox);  // 全体を囲うAABBを計算

 
												   // 領域分割をせず、polygons を含む葉ノードを構築する場合を暫定の bestCost にする
	float bestCost = T_tri * polygons.size();

	int bestAxis = -1;  // 分割に最も良い軸 (0:x, 1:y, 2:z)
	int bestSplitIndex = -1;  // 最も良い分割場所
	float SA_root = surfaceArea(node->bbox); // ノード全体のAABBの表面積
	for (int axis = 0; axis<3; axis++) {
		// ポリゴンリストを、それぞれのAABBの中心座標を使い、axis でソートする
		std::sort(polygons.begin(), polygons.end(),
			[axis](const TRIANGLE *a, const TRIANGLE *b) {
			double A = ((a->bbox[1][axis] + a->bbox[0][axis]) / 2);
			double B = ((b->bbox[1][axis] + b->bbox[0][axis]) / 2);

			return (A<B);//AABB の中心座標でソートx<y<zの優先順位？
		});
	
		std::vector<TRIANGLE *> s1, s2(polygons);  // 分割された2つの領域
		float s1bbox[2][3]; emptyAABB(s1bbox); // S1のAABB

											   // AABBの表面積リスト。s1SA[i], s2SA[i] は、
											   // 「S1側にi個、S2側に(polygons.size()-i)個ポリゴンがあるように分割」したときの表面積
		std::vector<float> s1SA(polygons.size() + 1, INF), s2SA(polygons.size() + 1, INF);

		// 可能な分割方法について、s1側の AABB の表面積を計算
		for (int i = 0; i <= polygons.size(); i++) {
			s1SA[i] = fabs(surfaceArea(s1bbox)); // 現在のS1のAABBの表面積を計算
			if (s2.size() > 0) {
				// s2側で、axis について最左 (最小位置) にいるポリゴンをS1の最右 (最大位置) に移す
				TRIANGLE *p = s2.front();
				s1.push_back(p); s2.erase(s2.begin());//TODOもしかしたら直す
				// 移したポリゴンのAABBをマージしてS1のAABBとする
				mergeAABB(s1bbox, p->bbox, s1bbox);
			}
		}

		// 逆にS2側のAABBの表面積を計算しつつ、SAH を計算
		float s2bbox[2][3]; emptyAABB(s2bbox);
		for (int i = polygons.size(); i >= 0; i--) {
			s2SA[i] = fabs(surfaceArea(s2bbox)); // 現在のS2のAABBの表面積を計算
			if (s1.size() > 0 && s2.size() > 0) {//s1sizeのみだとs2sizeが-になってもまわる
				// SAH-based cost の計算
				float cost =
					2 * T_aabb + (s1SA[i] * s1.size() + s2SA[i] * s2.size()) * T_tri / SA_root;
				// 最良コストが更新されたか？
				if (cost < bestCost) {
					bestCost = cost; bestAxis = axis; bestSplitIndex = i;
				}
			}

			if (s1.size() > 0) {
				// S1側で、axis について最右にいるポリゴンをS2の最左に移す
				TRIANGLE *p = s1.back();
				s2.insert(s2.begin(), p); s1.pop_back();
				// 移したポリゴンのAABBをマージしてS2のAABBとする
				mergeAABB(s2bbox, p->bbox, s2bbox);
			}
		}
	}

	if (bestAxis == -1) {
		// 現在のノードを葉ノードとするのが最も効率が良い結果になった
		// => 葉ノードの作成
		makeLeaf(polygons, node);
	}
	else {
		// bestAxis に基づき、左右に分割
		// bestAxis でソート
		std::sort(polygons.begin(), polygons.end(),
			[bestAxis](const TRIANGLE *a, const TRIANGLE *b) {
			return ((a->bbox[1][bestAxis]+a->bbox[0][bestAxis])/2) <((b->bbox[1][bestAxis]+b->bbox[0][bestAxis]) / 2);
		});
		// 左右の子ノードを作成
		used_node_count++;
		node->children[0] = used_node_count;
		used_node_count++;
		node->children[1] = used_node_count; 
		// ポリゴンリストを分割
		std::vector<TRIANGLE *> left(polygons.begin(), polygons.begin() + bestSplitIndex);
		std::vector<TRIANGLE *> right(polygons.begin() + bestSplitIndex, polygons.end());
		// 再帰処理
		constructBVH_internal(left, node->children[0]);
		constructBVH_internal(right, node->children[1]);
	}
}

// フロントエンド関数．これを呼べば nodes[0] をルートとした BVH が構築される
void constructBVH(std::vector<TRIANGLE *> &polygons) {
	
						//ポリゴン数が最大で2^()
	used_node_count = 0;
	constructBVH_internal(polygons, 0);  // nodes[0] をルートノードとみなす
}
#endif  // _BVH_H_

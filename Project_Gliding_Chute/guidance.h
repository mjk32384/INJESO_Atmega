/**
 * 현재 heading 방향의 방위각 계산
 * @param q 위성의 자세 쿼터니안
 * @param v 위성의 heading을 IMU 센서 좌표계로 나타낸 값
 * @return 목표 속도 벡터의 방위각
 */
float Theta1(float* q, float v[3]);

/**
 * GPS 데이터를 이용해서 목표 속도 벡터의 방위각 계산
 * @param v 목표 지점의 [위도, 경도]
 * @param w 현재 위성의 [위도, 경도]
 * @return 목표 속도 벡터의 방위각
 */
float Theta2(float v[2], float w[2]);
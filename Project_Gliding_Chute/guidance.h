/**
 * ���� heading ������ ������ ���
 * @param q ������ �ڼ� ���ʹϾ�
 * @param v ������ heading�� IMU ���� ��ǥ��� ��Ÿ�� ��
 * @return ��ǥ �ӵ� ������ ������
 */
float Theta1(float* q, float v[3]);

/**
 * GPS �����͸� �̿��ؼ� ��ǥ �ӵ� ������ ������ ���
 * @param v ��ǥ ������ [����, �浵]
 * @param w ���� ������ [����, �浵]
 * @return ��ǥ �ӵ� ������ ������
 */
float Theta2(float v[2], float w[2]);
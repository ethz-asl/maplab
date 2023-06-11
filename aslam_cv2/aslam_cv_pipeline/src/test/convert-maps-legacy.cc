#include <aslam/pipeline/test/convert-maps-legacy.h>

void aslam::convertMapsLegacy(cv::InputArray _map1, cv::InputArray _map2,
                       cv::OutputArray _dstmap1, cv::OutputArray _dstmap2,
                       int dstm1type, bool nninterpolate) {
  cv::Mat map1 = _map1.getMat(), map2 = _map2.getMat(), dstmap1, dstmap2;
  cv::Size size = map1.size();
  const cv::Mat *m1 = &map1, *m2 = &map2;
  int m1type = m1->type(), m2type = m2->type();

  CV_Assert((m1type == CV_16SC2 && (nninterpolate || m2type == CV_16UC1 || m2type == CV_16SC1)) ||
      (m2type == CV_16SC2 && (nninterpolate || m1type == CV_16UC1 || m1type == CV_16SC1)) ||
      (m1type == CV_32FC1 && m2type == CV_32FC1) ||
      (m1type == CV_32FC2 && !m2->data));

  if (m2type == CV_16SC2) {
    std::swap(m1, m2);
    std::swap(m1type, m2type);
  }

  if (dstm1type <= 0)
    dstm1type = m1type == CV_16SC2 ? CV_32FC2 : CV_16SC2;
  CV_Assert(dstm1type == CV_16SC2 || dstm1type == CV_32FC1 || dstm1type == CV_32FC2);
  _dstmap1.create(size, dstm1type);
  dstmap1 = _dstmap1.getMat();

  if (!nninterpolate && dstm1type != CV_32FC2) {
    _dstmap2.create(size, dstm1type == CV_16SC2 ? CV_16UC1 : CV_32FC1);
    dstmap2 = _dstmap2.getMat();
  } else
    _dstmap2.release();

  if (m1type == dstm1type || (nninterpolate &&
      ((m1type == CV_16SC2 && dstm1type == CV_32FC2) ||
          (m1type == CV_32FC2 && dstm1type == CV_16SC2)))) {
    m1->convertTo(dstmap1, dstmap1.type());
    if (dstmap2.data && dstmap2.type() == m2->type())
      m2->copyTo(dstmap2);
    return;
  }

  if (m1type == CV_32FC1 && dstm1type == CV_32FC2) {
    cv::Mat vdata[] = {*m1, *m2};
    merge(vdata, 2, dstmap1);
    return;
  }

  if (m1type == CV_32FC2 && dstm1type == CV_32FC1) {
    cv::Mat mv[] = {dstmap1, dstmap2};
    split(*m1, mv);
    return;
  }

  if (m1->isContinuous() && (!m2->data || m2->isContinuous()) &&
      dstmap1.isContinuous() && (!dstmap2.data || dstmap2.isContinuous())) {
    size.width *= size.height;
    size.height = 1;
  }

  const float scale = 1.f / cv::INTER_TAB_SIZE;
  int x, y;
  for (y = 0; y < size.height; y++) {
    const float *src1f = (const float *) (m1->data + m1->step * y);
    const float *src2f = (const float *) (m2->data + m2->step * y);
    const short *src1 = (const short *) src1f;
    const ushort *src2 = (const ushort *) src2f;

    float *dst1f = (float *) (dstmap1.data + dstmap1.step * y);
    float *dst2f = (float *) (dstmap2.data + dstmap2.step * y);
    short *dst1 = (short *) dst1f;
    ushort *dst2 = (ushort *) dst2f;

    if (m1type == CV_32FC1 && dstm1type == CV_16SC2) {
      if (nninterpolate)
        for (x = 0; x < size.width; x++) {
          dst1[x * 2] = cv::saturate_cast<short>(src1f[x]);
          dst1[x * 2 + 1] = cv::saturate_cast<short>(src2f[x]);
        }
      else
        for (x = 0; x < size.width; x++) {
          int ix = cv::saturate_cast<int>(src1f[x] * cv::INTER_TAB_SIZE);
          int iy = cv::saturate_cast<int>(src2f[x] * cv::INTER_TAB_SIZE);
          dst1[x * 2] = cv::saturate_cast<short>(ix >> cv::INTER_BITS);
          dst1[x * 2 + 1] = cv::saturate_cast<short>(iy >> cv::INTER_BITS);
          dst2[x] = (ushort) ((iy & (cv::INTER_TAB_SIZE - 1)) * cv::INTER_TAB_SIZE + (ix & (cv::INTER_TAB_SIZE - 1)));
        }
    } else if (m1type == CV_32FC2 && dstm1type == CV_16SC2) {
      if (nninterpolate)
        for (x = 0; x < size.width; x++) {
          dst1[x * 2] = cv::saturate_cast<short>(src1f[x * 2]);
          dst1[x * 2 + 1] = cv::saturate_cast<short>(src1f[x * 2 + 1]);
        }
      else
        for (x = 0; x < size.width; x++) {
          int ix = cv::saturate_cast<int>(src1f[x * 2] * cv::INTER_TAB_SIZE);
          int iy = cv::saturate_cast<int>(src1f[x * 2 + 1] * cv::INTER_TAB_SIZE);
          dst1[x * 2] = cv::saturate_cast<short>(ix >> cv::INTER_BITS);
          dst1[x * 2 + 1] = cv::saturate_cast<short>(iy >> cv::INTER_BITS);
          dst2[x] = (ushort) ((iy & (cv::INTER_TAB_SIZE - 1)) * cv::INTER_TAB_SIZE + (ix & (cv::INTER_TAB_SIZE - 1)));
        }
    } else if (m1type == CV_16SC2 && dstm1type == CV_32FC1) {
      for (x = 0; x < size.width; x++) {
        int fxy = src2 ? src2[x] & (cv::INTER_TAB_SIZE2 - 1) : 0;
        dst1f[x] = src1[x * 2] + (fxy & (cv::INTER_TAB_SIZE - 1)) * scale;
        dst2f[x] = src1[x * 2 + 1] + (fxy >> cv::INTER_BITS) * scale;
      }
    } else if (m1type == CV_16SC2 && dstm1type == CV_32FC2) {
      for (x = 0; x < size.width; x++) {
        int fxy = src2 ? src2[x] & (cv::INTER_TAB_SIZE2 - 1) : 0;
        dst1f[x * 2] = src1[x * 2] + (fxy & (cv::INTER_TAB_SIZE - 1)) * scale;
        dst1f[x * 2 + 1] = src1[x * 2 + 1] + (fxy >> cv::INTER_BITS) * scale;
      }
    } else
      CV_Error(CV_StsNotImplemented, "Unsupported combination of input/output matrices");
  }
}


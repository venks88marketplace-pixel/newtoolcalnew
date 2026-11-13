#include "IACalibrator.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include "headers/calibrationStorage.h"
#include <QImage>
#include <fstream>
#include <qdebug.h>
#include "DMD/blobfinder.h"

IACalibrator::IACalibrator(cv::Size patternSize, std::filesystem::path analysisFolder, QObject *parent)
    : QObject{parent},
    m_patternSize{patternSize},
    m_analysisFolder{analysisFolder} {
}

cv::Vec3d calculateMean(const cv::Mat_<cv::Vec3d> &points) {
    cv::Mat_<cv::Vec3d> result;
    cv::reduce(points, result, 0, cv::REDUCE_AVG);
    return result(0, 0);
}

cv::Mat_<double> findRigidTransform(const cv::Mat_<cv::Vec3d> &points1, const cv::Mat_<cv::Vec3d> points2) {
    /* Calculate centroids. */
    cv::Vec3d t1 = -calculateMean(points1);
    cv::Vec3d t2 = -calculateMean(points2);

    cv::Mat_<double> T1 = cv::Mat_<double>::eye(4, 4);
    T1(0, 3) = t1[0];
    T1(1, 3) = t1[1];
    T1(2, 3) = t1[2];

    cv::Mat_<double> T2 = cv::Mat_<double>::eye(4, 4);
    T2(0, 3) = -t2[0];
    T2(1, 3) = -t2[1];
    T2(2, 3) = -t2[2];

    /* Calculate covariance matrix for input points. Also calculate RMS deviation from centroid
     * which is used for scale calculation.         */
    cv::Mat_<double> C(3, 3, 0.0);
    double p1Rms = 0, p2Rms = 0;
    for (int ptIdx = 0; ptIdx < points1.rows; ptIdx++) {
        cv::Vec3d p1 = points1(ptIdx, 0) + t1;
        cv::Vec3d p2 = points2(ptIdx, 0) + t2;
        p1Rms += p1.dot(p1);
        p2Rms += p2.dot(p2);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                C(i, j) += p2[i] * p1[j];
            }
        }
    }
    cv::Mat_<double> u, s, vh;
    cv::SVD::compute(C, s, u, vh);
    cv::Mat_<double> R = u * vh;
    if (cv::determinant(R) < 0) {
        R -= u.col(2) * (vh.row(2) * 2.0);
    }

    double scale = sqrt(p2Rms / p1Rms);
    R *= scale;

    cv::Mat_<double> M = cv::Mat_<double>::eye(4, 4);
    R.copyTo(M.colRange(0, 3).rowRange(0, 3));

    cv::Mat_<double> result = T2 * M * T1;
    result /= result(3, 3);

    return result.rowRange(0, 3);
}

void IACalibrator::na2ia() {
    cv::Mat_<cv::Vec3d> src(5,1), dst(5,1);
    dst.row(0) = cv::Vec3d{ 0, 0, 0};
    for (int i = 1; i<5; ++i) {
        auto w = m_cbFrameFlat[i-1];
        auto n = cv::norm(w);
        auto n2 = sqrt(w.x*w.x + w.y*w.y);
        dst.row(i) = cv::Vec3d{ w.x * n/n2, w.y *n/n2, 0};
        //cout << w << " " << dst.row(i) << std::endl;
    }
    src.row(0) = cv::Vec3d{m_center.x, m_center.y, m_center.z};
    for (int i = 1; i<5; ++i) {
        auto w = m_cbFrame[i-1];
        src.row(i) = cv::Vec3d{w.x, w.y, w.z};
    }


    m_toIA = findRigidTransform(src, dst);
    //std::cout << nfov.toIA << std::endl;
    std::vector<cv::Point3d> xformed;
    for (int i = 0; i<5; ++i) {
        auto w = src.row(i);
        std::cout << "w " << w;
        auto w1 = cv::Vec4d(w.at<cv::Vec3d>(0)[0],w.at<cv::Vec3d>(0)[1],w.at<cv::Vec3d>(0)[2], 1.0);
        cv::Mat p;
        cv::transform(src.row(i), p, m_toIA);
        double *q = p.ptr<double>(0);
        xformed.push_back({*q, *(q+1), *(q+2)});
        //cout << p.t() << " " << xformed.back() << " " << dst.row(i) << std::endl;
        //cout << (nfov.toIA *  w1).t() << " " << dst.row(i) << std::endl;
    }
    if (m_analysisFolder.empty() == false) {
        std::ofstream fid(m_analysisFolder / "nfov3d_transformed.csv");
        fid << "Checkerboard corner and DMD center after transform to IA." << std::endl;
        for (auto yy : xformed) {
            fid << yy << std::endl;
        }
        fid.close();
    }
}

bool getCorners(cv::Size patternSize, const cv::Mat& img, std::array<cv::Point2f, 4>& corners) {
    std::vector<cv::Point2f> cbPoints;
    bool found = cv::findChessboardCornersSB(img, patternSize, cbPoints, 0);
    if (found && cbPoints.size() == patternSize.area()) {
        cv::cornerSubPix(img, cbPoints, cv::Size(11, 11),
                         cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

        float m00 = std::numeric_limits<float>::max();
        float m01 = std::numeric_limits<float>::max();
        float m10 = std::numeric_limits<float>::min();
        float m11 = std::numeric_limits<float>::min();
        for (auto &f : cbPoints) {
            float sum = abs(f.x + f.y);
            float dif = f.x - f.y;
            if ( sum < m00) {
                m00 = sum;
                corners[0] = f;
            }
            if (sum > m11) {
                m11 = sum;
                corners[2] = f;
            }
            if (dif > m10) {
                m10 = dif;
                corners[1] = f;
            }
            if (dif < m01) {
                m01 = dif;
                corners[3] = f;
            }
        }

        //for(auto& p : corners){
        //    cv::circle(img, p, 5, cv::Scalar(0,0,0), 2 );
        //}
        //cv::imshow("Chessboard Corners", img);
        //cv::waitKey(0);

        return true;
    }
    return false;
}

bool IACalibrator::getCheckerboardCorners(const cv::Mat& imgLeft, const cv::Mat& imgRight){
    bool foundLeft = getCorners(m_patternSize, imgLeft, m_checkerCornersLeft);
    bool foundRight = getCorners(m_patternSize, imgRight, m_checkerCornersRight);
    return foundLeft and foundRight;
}

bool IACalibrator::trainIA(const cv::Mat& imgLeft, const cv::Mat& imgRight){
    auto leftDots = BlobFinder::findBlobs(imgLeft);
    auto rightDots = BlobFinder::findBlobs(imgRight);
    if(leftDots.size() == 1 && rightDots.size() == 1){

        std::vector<cv::Point2f> x1, x2;
        x1.push_back(leftDots[0]); x2.push_back(rightDots[0]);
        for (int i=0; i<4; ++i) {
            x1.push_back(m_checkerCornersLeft[i]); x2.push_back(m_checkerCornersRight[i]);
        }
        std::vector<cv::Point2f> ux1, ux2;

        calibrationStorage cal;
        cv::Mat K1, K2;
        cv::Mat D1, D2;
        cv::Mat R, T, E, F;
        cv::Mat R1, R2, P1, P2, Q;

        bool calOK1 = cal.getNfovCameraMatrices(D1, K1, D2, K2);
        bool calOK2 = cal.getStereoMatrix(F, E, R, T, R1, R2, P1, P2, Q);
        if (calOK1 == false || calOK2 == false) {
            std::cout << "Calibration error " << std::endl;
            return false;
        }

        cv::undistortPoints(x1, ux1, K1, D1, R1, P1);
        cv::undistortPoints(x2, ux2, K2, D2, R2, P2);
        std::vector<cv::Point3f> X;
        for (int i=0; i<5; ++i) {
            X.push_back({ux1[i].x, (ux1[i].y + ux2[i].y) / 2, (ux1[i].x - ux2[i].x)});
        }
        std::vector<cv::Point3f> y;
        cv::perspectiveTransform(X, y, Q);

        if (m_analysisFolder.empty() == false) {
            std::ofstream fid(m_analysisFolder / "nfov3d.csv");
            fid << "Checkerboard corners and DMD center. " << std::endl;
            for (auto yy : y) {
                fid << yy << std::endl;
            }
            fid.close();
        }

        m_center = y[0];
        m_cbFrame.resize(4);
        m_cbFrameFlat.resize(4);
        m_cbFrame[0] = y[1];
        m_cbFrame[1] = y[2];
        m_cbFrame[2] = y[3];
        m_cbFrame[3] = y[4];
        m_cbFrameFlat[0] = y[1] - y[0];
        m_cbFrameFlat[1] = y[2] - y[0];
        m_cbFrameFlat[2] = y[3] - y[0];
        m_cbFrameFlat[3] = y[4] - y[0];

        na2ia();

        cal.updateInstrumentAxis(m_toIA, 0.0, 0.0, 0.0);
        //cal.updateInstrumentAxis(nfov.toIA, wIAcenter.x, wIAcenter.y, wIADepth);
        std::cout << "Camera to Instrument Axis transform:" << std::endl;
        std::cout << m_toIA << std::endl;
        std::cout << "Instrument axes calibration success." << std::endl;

    }
    else{
        return false;
    }
    return true;
}

QImage IACalibrator::createImage(int radius){

    cv::Mat img = cv::Mat::zeros(768, 1024, CV_8UC1);
    cv::Point2f center(img.cols/2, img.rows/2);
    cv::ellipse(img, center, cv::Size(radius, radius), 0, 0, 360, cv::Scalar(255), -1);

    //cv::imshow("DMD image", img);
    //cv::waitKey(0);
    QImage qimg(img.data, img.cols, img.rows, static_cast<int>(img.step), QImage::Format_Grayscale8);
    return qimg.copy();
 }


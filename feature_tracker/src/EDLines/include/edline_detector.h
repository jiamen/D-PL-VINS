#ifndef __EDLINEDETECTOR_PARALLEL_HH_
#define __EDLINEDETECTOR_PARALLEL_HH_


#include <array>
#include <iostream>
#include <list>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <vector>

#include "line.h"

struct Pixel
{
    unsigned int x;  //X coordinate
    unsigned int y;  //Y coordinate
};

// 边缘线的点的X、Y集合
struct EdgeChains
{
    std::vector<unsigned int> xCors;  // all the x coordinates of edge points
    std::vector<unsigned int> yCors;  // all the y coordinates of edge points
    std::vector<unsigned int> sId;    // the start index of each edge in the coordinate arrays
    unsigned int numOfEdges;          // the number of edges whose length are larger than minLineLen; numOfEdges < sId.size;
};

struct LineChains
{
    std::vector<unsigned int> xCors;  // all the x coordinates of line points
    std::vector<unsigned int> yCors;  // all the y coordinates of line points
    std::vector<unsigned int> sId;    // the start index of each line in the coordinate arrays
    unsigned int numOfLines;          // the number of lines whose length are larger than minLineLen; numOfLines < sId.size;
};

typedef std::list<Pixel> PixelChain;        // each edge is a pixel chain.


// EDLine线特征检测过程中的相关参数设置
struct EDLineParam
{
    int ksize;                  //
    float sigma;                //
    float gradientThreshold;    // 梯度阈值

    float anchorThreshold;
    int scanIntervals;
    int minLineLen;
    double lineFitErrThreshold;
};


#define RELATIVE_ERROR_FACTOR 100.0
#define M_LN10 2.30258509299404568402
#define log_gamma(x) ((x) > 15.0 ? log_gamma_windschitl(x) : log_gamma_lanczos(x))

/* This class is used to detect lines from input image.
 * First, edges are extracted from input image following the method presented in Cihan Topal and
 * Cuneyt Akinlar's paper:"Edge Drawing: A Heuristic Approach to Robust Real-Time Edge Detection", 2010.
 * Then, lines are extracted from the edge image following the method presented in Cuneyt Akinlar and
 * Cihan Topal's paper:"EDLines: A real-time line segment detector with a false detection control", 2011
 * PS: The linking step of edge detection has a little bit difference with the Edge drawing algorithm
 *     described in the paper. The edge chain doesn't stop when the pixel direction is changed.
 */
class EDLineDetector
{
public:
    EDLineDetector();
    EDLineDetector(EDLineParam param);

    ~EDLineDetector();

    /*extract edges from image
	 *image:    In, gray image;
	 *edges:    Out, store the edges, each edge is a pixel chain
	 *smoothed: In, flag to mark whether the image has already been smoothed by Gaussian filter.
	 *return -1: error happen
	 */
    int EdgeDrawing(cv::Mat& image, EdgeChains& edgeChains, bool smoothed=false);
    /*extract lines from image
	 *image:    In, gray image;
	 *lines:    Out, store the extracted lines,
	 *smoothed: In, flag to mark whether the image has already been smoothed by Gaussian filter.
	 *return -1: error happen
	 */
    // int EDline(cv::Mat &image, LineChains &lines, bool smoothed = false);
    /*extract line from image, and store them*/
    // int EDline(cv::Mat &image, bool smoothed = false);

    //My version

    int EDline(cv::Mat& image,
               std::vector<Line1>& lines,
               bool smoothed=false);

    cv::Mat dxImg_;     // store the dxImg;
    cv::Mat dyImg_;     // store the dyImg;
    cv::Mat gImgWO_;    //store the gradient image without threshold;
    // LineChains lines_;  //store the detected line chains;


    cv::Mat gImg_;    //store the gradient image;
    cv::Mat dirImg_;  //store the direction image

    cv::Mat dxABS_m_;
    cv::Mat dyABS_m_;
    cv::Mat sumDxDy_;


    //store the line Equation coefficients, vec3=[w1,w2,w3] for line w1*x + w2*y + w3=0;
    std::vector<std::array<double, 3> > lineEquations_;
    //store the line endpoints, [x1,y1,x2,y3]
    std::vector<std::array<float, 4> > lineEndpoints_;
    //store the line direction
    std::vector<float> lineDirection_;
    //store the line salience, which is the summation of gradients of pixels on line
    std::vector<float> lineSalience_;
    unsigned int imageWidth;
    unsigned int imageHeight;

    // float abs_time_sum_ = 0.0;

private:
    void InitEDLine_();

    EdgeChains edges_;

    int ksize_;
    float sigma_;  //the sigma of Gaussian kernal, default value is 1.0.
    /*the threshold of pixel gradient magnitude.
	 *Only those pixel whose gradient magnitude are larger than this threshold will be
	 *taken as possible edge points. Default value is 36*/
    short gradienThreshold_;
    /*If the pixel's gradient value is bigger than both of its neighbors by a
	 *certain threshold (ANCHOR_THRESHOLD), the pixel is marked to be an anchor.
	 *Default value is 8*/
    unsigned char anchorThreshold_;
    /*anchor testing can be performed at different scan intervals, i.e.,
	 *every row/column, every second row/column etc.
	 *Default value is 2*/
    unsigned int scanIntervals_;
    int minLineLen_;  //minimal acceptable line length
    /*For example, there two edges in the image:
	 *edge1 = [(7,4), (8,5), (9,6),| (10,7)|, (11, 8), (12,9)] and
	 *edge2 = [(14,9), (15,10), (16,11), (17,12),| (18, 13)|, (19,14)] ; then we store them as following:
	 *pFirstPartEdgeX_ = [10, 11, 12, 18, 19];//store the first part of each edge[from middle to end]
	 *pFirstPartEdgeY_ = [7,  8,  9,  13, 14];
	 *pFirstPartEdgeS_ = [0,3,5];// the index of start point of first part of each edge
	 *pSecondPartEdgeX_ = [10, 9, 8, 7, 18, 17, 16, 15, 14];//store the second part of each edge[from middle to front]
	 *pSecondPartEdgeY_ = [7,  6, 5, 4, 13, 12, 11, 10, 9];//anchor points(10, 7) and (18, 13) are stored again
	 *pSecondPartEdgeS_ = [0, 4, 9];// the index of start point of second part of each edge
	 *This type of storage order is because of the order of edge detection process.
	 *For each edge, start from one anchor point, first go right, then go left or first go down, then go up*/
    unsigned int* pFirstPartEdgeX_;   //store the X coordinates of the first part of the pixels for chains
    unsigned int* pFirstPartEdgeY_;   //store the Y coordinates of the first part of the pixels for chains
    unsigned int* pFirstPartEdgeS_;   //store the start index of every edge chain in the first part arrays
    unsigned int* pSecondPartEdgeX_;  //store the X coordinates of the second part of the pixels for chains
    unsigned int* pSecondPartEdgeY_;  //store the Y coordinates of the second part of the pixels for chains
    unsigned int* pSecondPartEdgeS_;  //store the start index of every edge chain in the second part arrays
    unsigned int* pAnchorX_;          //store the X coordinates of anchors
    unsigned int* pAnchorY_;          //store the Y coordinates of anchors

    cv::Mat edgeImage_;

    /*The threshold of line fit error;
	 *If lineFitErr is large than this threshold, then
	 *the pixel chain is not accepted as a single line segment.*/
    double lineFitErrThreshold_;

    cv::Mat image_;
    cv::Mat zeroMat_;

    // std::mutex g_lock_;

    double logNT_;

    // void AbsAdd(const cv::Mat &src1, const cv::Mat &src2, cv::Mat &dst);


    /**  Compare doubles by relative error.  处理四舍五入误差
	     The resulting rounding error after floating point computations
	     depend on the specific operations done. The same number computed by
	     different algorithms could present different rounding errors. For a
	     useful comparison, an estimation of the relative rounding error
	     should be considered and compared to a factor times EPS. The factor
	     should be related to the accumulated rounding error in the chain of
	     computation. Here, as a simplification, a fixed factor is used.
	 */
    static int double_equal(double a, double b)
    {
        double abs_diff, aa, bb, abs_max;

        /* trivial case：不重要的情况 */
        if (a == b)
            return true;

        abs_diff = fabs(a - b);

        aa = fabs(a);
        bb = fabs(b);
        abs_max = aa > bb ? aa : bb;
        /* DBL_MIN is the smallest normalized number, thus, the smallest
	      number whose relative error is bounded by DBL_EPSILON. For
	      smaller numbers, the same quantization steps as for DBL_MIN
	      are used. Then, for smaller numbers, a meaningful "relative"
	      error should be computed by dividing the difference by DBL_MIN. */
        if (abs_max < DBL_MIN) abs_max = DBL_MIN;
        /* equal if relative error <= factor x eps */
        return (abs_diff / abs_max) <= (RELATIVE_ERROR_FACTOR * DBL_EPSILON);
    }


    /** Computes the natural logarithm of the absolute value of
	     the gamma function of x using the Lanczos approximation.
	     // 使用 Lanczos 近似计算 x 的 gamma 函数的绝对值的自然对数。
	     See http://www.rskey.org/gamma.htm
	     The formula used is
	     @f[
	       \Gamma(x) = \frac{ \sum_{n=0}^{N} q_n x^n }{ \Pi_{n=0}^{N} (x+n) }
	                   (x+5.5)^{x+0.5} e^{-(x+5.5)}
	     @f]
	     so
	     @f[
	       \log\Gamma(x) = \log\left( \sum_{n=0}^{N} q_n x^n \right)
	                       + (x+0.5) \log(x+5.5) - (x+5.5) - \sum_{n=0}^{N} \log(x+n)
	     @f]
	     and
	       q0 = 75122.6331530,
	       q1 = 80916.6278952,
	       q2 = 36308.2951477,
	       q3 = 8687.24529705,
	       q4 = 1168.92649479,
	       q5 = 83.8676043424,
	       q6 = 2.50662827511.
	 */
    static double log_gamma_lanczos(double x)
    {
        static double q[7] = {75122.6331530, 80916.6278952, 36308.2951477,
                              8687.24529705, 1168.92649479, 83.8676043424,
                              2.50662827511};
        double a = (x + 0.5) * log(x + 5.5) - (x + 5.5);
        double b = 0.0;
        int n;
        for (n = 0; n < 7; n++) {
            a -= log(x + (double)n);
            b += q[n] * pow(x, (double)n);
        }
        return a + log(b);
    }


    /** Computes the natural logarithm of the absolute value of
	     the gamma function of x using Windschitl method.
	     See http://www.rskey.org/gamma.htm
	     The formula used is
	     @f[
	         \Gamma(x) = \sqrt{\frac{2\pi}{x}} \left( \frac{x}{e}
	                     \sqrt{ x\sinh(1/x) + \frac{1}{810x^6} } \right)^x
	     @f]
	     so
	     @f[
	         \log\Gamma(x) = 0.5\log(2\pi) + (x-0.5)\log(x) - x
	                       + 0.5x\log\left( x\sinh(1/x) + \frac{1}{810x^6} \right).
	     @f]
	     This formula is a good approximation when x > 15.
	 */
    static double log_gamma_windschitl(double x)
    {
        return 0.918938533204673 + (x - 0.5) * log(x) - x + 0.5 * x * log(x * sinh(1 / x) + 1 / (810.0 * pow(x, 6.0)));
    }


    /** Computes -log10(NFA).
	     NFA stands for Number of False Alarms:
	     @f[
	         \mathrm{NFA} = NT \cdot B(n,k,p)
	     @f]
	     - NT       - number of tests
	     - B(n,k,p) - tail of binomial distribution with parameters n,k and p:
	     @f[
	         B(n,k,p) = \sum_{j=k}^n
	                    \left(\begin{array}{c}n\\j\end{array}\right)
	                    p^{j} (1-p)^{n-j}
	     @f]
	     The value -log10(NFA) is equivalent but more intuitive than NFA:
	     - -1 corresponds to 10 mean false alarms
	     -  0 corresponds to 1 mean false alarm
	     -  1 corresponds to 0.1 mean false alarms
	     -  2 corresponds to 0.01 mean false alarms
	     -  ...
	     Used this way, the bigger the value, better the detection,
	     and a logarithmic scale is used.
	     @param n,k,p binomial parameters.
	     @param logNT logarithm of Number of Tests
	     The computation is based in the gamma function by the following
	     relation:
	     @f[
	         \left(\begin{array}{c}n\\k\end{array}\right)
	         = \frac{ \Gamma(n+1) }{ \Gamma(k+1) \cdot \Gamma(n-k+1) }.
	     @f]
	     We use efficient algorithms to compute the logarithm of
	     the gamma function.
	     To make the computation faster, not all the sum is computed, part
	     of the terms are neglected based on a bound to the error obtained
	     (an error of 10% in the result is accepted).
	 */
    static double nfa(int n, int k, double p, double logNT)
    {
        double tolerance = 0.1; /* an error of 10% in the result is accepted */
        double log1term, term, bin_term, mult_term, bin_tail, err, p_term;
        int i;

        /* check parameters */
        if (n < 0 || k < 0 || k > n || p <= 0.0 || p >= 1.0) {
            std::cout << "nfa: wrong n, k or p values." << std::endl;
            exit(0);
        }
        /* trivial cases */
        if (n == 0 || k == 0) return -logNT;
        if (n == k) return -logNT - (double)n * log10(p);

        /* probability term */
        p_term = p / (1.0 - p);

        /* compute the first term of the series */
        /*
	      binomial_tail(n,k,p) = sum_{i=k}^n bincoef(n,i) * p^i * (1-p)^{n-i}
	      where bincoef(n,i) are the binomial coefficients.
	      But
	        bincoef(n,k) = gamma(n+1) / ( gamma(k+1) * gamma(n-k+1) ).
	      We use this to compute the first term. Actually the log of it.
		 */
        log1term = log_gamma((double)n + 1.0) - log_gamma((double)k + 1.0) - log_gamma((double)(n - k) + 1.0) + (double)k * log(p) + (double)(n - k) * log(1.0 - p);
        term = exp(log1term);

        /* in some cases no more computations are needed */
        if (double_equal(term, 0.0)) {             /* the first term is almost zero */
            if ((double)k > (double)n * p)         /* at begin or end of the tail?  */
                return -log1term / M_LN10 - logNT; /* end: use just the first term  */
            else
                return -logNT; /* begin: the tail is roughly 1  */
        }

        /* compute more terms if needed */
        bin_tail = term;
        for (i = k + 1; i <= n; i++) {
            /*    As
	            term_i = bincoef(n,i) * p^i * (1-p)^(n-i)
	          and
	            bincoef(n,i)/bincoef(n,i-1) = n-i+1 / i,
	          then,
	            term_i / term_i-1 = (n-i+1)/i * p/(1-p)
	          and
	            term_i = term_i-1 * (n-i+1)/i * p/(1-p).
	          p/(1-p) is computed only once and stored in 'p_term'.
			 */
            bin_term = (double)(n - i + 1) / (double)i;
            mult_term = bin_term * p_term;
            term *= mult_term;
            bin_tail += term;
            if (bin_term < 1.0) {
                /* When bin_term<1 then mult_term_j<mult_term_i for j>i.
	              Then, the error on the binomial tail when truncated at
	              the i term can be bounded by a geometric series of form
	              term_i * sum mult_term_i^j.                            */
                err = term * ((1.0 - pow(mult_term, (double)(n - i + 1))) /
                              (1.0 - mult_term) -
                              1.0);
                /* One wants an error at most of tolerance*final_result, or:
	              tolerance * abs(-log10(bin_tail)-logNT).
	              Now, the error that can be accepted on bin_tail is
	              given by tolerance*final_result divided by the derivative
	              of -log10(x) when x=bin_tail. that is:
	              tolerance * abs(-log10(bin_tail)-logNT) / (1/bin_tail)
	              Finally, we truncate the tail if the error is less than:
	              tolerance * abs(-log10(bin_tail)-logNT) * bin_tail        */
                if (err < tolerance * fabs(-log10(bin_tail) - logNT) * bin_tail) break;
            }
        }
        return -log10(bin_tail) - logNT;
    }


    class EDLineDetectorParallel : public cv::ParallelLoopBody
    {
    public:
        EDLineDetectorParallel(EDLineDetector *_edline,
                               EdgeChains *_edges,
                //    unsigned char *_pdirImg,
                //    unsigned int *_pEdgeXCors,
                //    unsigned int *_pEdgeYCors,
                //    unsigned int *_pEdgeSID,
                               double _logNT,
                               cv::Mutex &_lock,
                               std::vector<Line1> *_lines);

        EDLineDetectorParallel &operator=(const EDLineDetectorParallel &)
        {
            return *this;
        };

        virtual void operator()(const cv::Range &range) const;

        /*For an input edge chain, find the best fit line, the default chain length is minLineLen_
	 *xCors:  In, pointer to the X coordinates of pixel chain;
	 *yCors:  In, pointer to the Y coordinates of pixel chain;
	 *offsetS:In, start index of this chain in array;
	 *lineEquation: Out, [a,b] which are the coefficient of lines y=ax+b(horizontal) or x=ay+b(vertical);
	 *return:  line fit error; -1:error happens;
	 */
        double LeastSquaresLineFit(cv::Mat_<float> &ATA, cv::Mat_<float> &ATV, cv::Mat_<float> &fitMatT, cv::Mat_<float> &fitVec,
                                   unsigned int *xCors, unsigned int *yCors,
                                   unsigned int offsetS, std::array<double, 2> &lineEquation) const;
        /*For an input pixel chain, find the best fit line. Only do the update based on new points.
	 *For A*x=v,  Least square estimation of x = Inv(A^T * A) * (A^T * v);
	 *If some new observations are added, i.e, [A; A'] * x = [v; v'],
	 *then x' = Inv(A^T * A + (A')^T * A') * (A^T * v + (A')^T * v');
	 *xCors:  In, pointer to the X coordinates of pixel chain;
	 *yCors:  In, pointer to the Y coordinates of pixel chain;
	 *offsetS:In, start index of this chain in array;
	 *newOffsetS: In, start index of extended part;
	 *offsetE:In, end index of this chain in array;
	 *lineEquation: Out, [a,b] which are the coefficient of lines y=ax+b(horizontal) or x=ay+b(vertical);
	 *return:  line fit error; -1:error happens;
	 */
        double LeastSquaresLineFit(cv::Mat_<float> &ATA, cv::Mat_<float> &ATV, cv::Mat_<float> &tempMatLineFit, cv::Mat_<float> &tempVecLineFit,
                                   unsigned int *xCors, unsigned int *yCors,
                                   unsigned int offsetS, unsigned int newOffsetS,
                                   unsigned int offsetE, std::array<double, 2> &lineEquation) const;
        /* Validate line based on the Helmholtz principle, which basically states that
	 * for a structure to be perceptually meaningful, the expectation of this structure
	 * by chance must be very low.
	 */
        bool LineValidation(unsigned int *xCors, unsigned int *yCors,
                            unsigned int offsetS, unsigned int offsetE,
                            std::array<double, 3> &lineEquation, float &direction) const;

    private:
        EDLineDetector *edline;
        EdgeChains *edges;
        int minLineLen;
        double lineFitErrThreshold;
        unsigned int imageWidth;
        unsigned int imageHeight;
        // unsigned char *pdirImg;
        unsigned int* pEdgeXCors;
        unsigned int* pEdgeYCors;
        unsigned int* pEdgeSID;
        double logNT;
        cv::Mutex &lock;
        std::vector<Line1> *lines;      // 最终提取到的线特征

        bool bValidate;  //flag to decide whether line will be validated

        cv::Mat dirImg;  //store the direction image
        cv::Mat dxImg;   //store the dxImg;
        cv::Mat dyImg;   //store the dyImg;
    };
};



#endif
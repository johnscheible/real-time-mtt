/*
 * Software License Agreement (BSD License)
 * 
 * Copyright (c)  2012, Wongun Choi
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies, 
 * either expressed or implied, of the FreeBSD Project.
 */

#include <iostream>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include "observation/roihog.h"

/* == HOG modification HOGCache is just copied from opencv implementation == */
#ifdef HAVE_CUDA
void ROIHOGDetector::detectMultiScaleROI(const cv::Mat& img, 
															CV_OUT std::vector<cv::Rect>& foundLocations,
															std::vector<DetectionROI>& locations,
															double hitThreshold,
															int groupThreshold)
{
	cv::gpu::GpuMat gpu_img(img);
	std::vector<cv::gpu::HOGConfidence> confidences;
	
	for(size_t i = 0; i < locations.size(); i++)
	{
		cv::gpu::HOGConfidence conf;
		conf.scale = locations[i].scale;
		
		locations[i].locations.clear();
		locations[i].confidences.clear();
		locations[i].part_score[0].clear();
		locations[i].part_score[1].clear();
		locations[i].part_score[2].clear();
		locations[i].part_score[3].clear();

		confidences.push_back(conf);
	}

	foundLocations.clear();
#if 0
	cv::gpu::GpuMat descriptors;
	impl_.getDescriptors(gpu_img, cv::Size(8,8), descriptors, cv::gpu::HOGDescriptor::DESCR_FORMAT_ROW_BY_ROW);
	
	cv::Mat temp;
	descriptors.download(temp);
	
	printf("size %d %d\n", temp.rows, temp.cols);

	for(int i = 0; i < temp.rows; i++)
	{
		double res = 0;
		for(int j = 0; j < temp.cols; j++) 
		{
			CV_Assert((temp.at<float>(i, j)) >= 0.0f);
			CV_Assert(!isnan(detector_[j]));
			CV_Assert(!isnan(temp.at<float>(i, j)));

			res += detector_[j] * temp.at<float>(i, j);
		}
		res += detector_[temp.cols];
		printf("%d:%.02f\t", i, res);
	}
	printf("\n");
	exit(1);
#else
	impl_.computeConfidenceMultiScale(gpu_img, foundLocations, hitThreshold, cv::Size(8, 8), cv::Size(0, 0), confidences, groupThreshold);
	// impl_.detectMultiScale(gpu_img, foundLocations, hitThreshold, cv::Size(8, 8), cv::Size(0, 0), 1.05, groupThreshold);
#endif
	for(size_t i = 0; i < confidences.size(); i++)
	{
		for(size_t j = 0; j < confidences[i].locations.size(); j++)
		{
			locations[i].locations.push_back(confidences[i].locations[j]);
			locations[i].confidences.push_back(confidences[i].confidences[j]);
			locations[i].part_score[0].push_back(0);
			locations[i].part_score[1].push_back(0);
			locations[i].part_score[2].push_back(0);
			locations[i].part_score[3].push_back(0);
		}
	}
}
#if 0
void ROIHOGDescriptor::detect(const GpuMat& img, vector<Point>& hits, double hit_threshold, 
                                    Size win_stride, Size padding)
{
    CV_Assert(img.type() == CV_8UC1 || img.type() == CV_8UC4);
    CV_Assert(padding == Size(0, 0));

    hits.clear();
    if (detector.empty())
        return;

    computeBlockHistograms(img);

    if (win_stride == Size())
        win_stride = block_stride;
    else
        CV_Assert(win_stride.width % block_stride.width == 0 &&
                  win_stride.height % block_stride.height == 0);

    Size wins_per_img = numPartsWithin(img.size(), win_size, win_stride);
#if 1
    labels.create(1, wins_per_img.area(), CV_32F);
#else
    labels.create(1, wins_per_img.area(), CV_8U);
#endif

    cv::gpu::hog::classify_hists(win_size.height, win_size.width, block_stride.height, block_stride.width, 
                        win_stride.height, win_stride.width, img.rows, img.cols, block_hists.ptr<float>(), 
#if 1 
												detector.ptr<float>(), (float)free_coef, (float)hit_threshold, labels.ptr<float>());
#else
                        detector.ptr<float>(), (float)free_coef, (float)hit_threshold, labels.ptr());
#endif

    labels.download(labels_host);
#if 1
    float* vec = labels_host.ptr<float>();
#else
    unsigned char* vec = labels_host.ptr();
#endif
    for (int i = 0; i < wins_per_img.area(); i++)
    {
        int y = i / wins_per_img.width;
        int x = i - wins_per_img.width * y;
#if 1
        if (vec[i] >= hit_threshold) {
            hits.push_back(Point(x * win_stride.width, y * win_stride.height));
				}
#else
        if (vec[i]) 
            hits.push_back(Point(x * win_stride.width, y * win_stride.height));
#endif
    }
}


void ROIHOGDescriptor::detectMultiScale(const GpuMat& img, vector<Rect>& found_locations, 
                                              double hit_threshold, Size win_stride, Size padding, 
                                              double scale0, int group_threshold)
{
    CV_Assert(img.type() == CV_8UC1 || img.type() == CV_8UC4);

    vector<double> level_scale;
    double scale = 1.;
    int levels = 0;

    for (levels = 0; levels < nlevels; levels++)
    {
        level_scale.push_back(scale);
        if (cvRound(img.cols/scale) < win_size.width || 
            cvRound(img.rows/scale) < win_size.height || scale0 <= 1)
            break;
        scale *= scale0;
    }
    levels = std::max(levels, 1);
    level_scale.resize(levels);

    std::vector<Rect> all_candidates;   
    vector<Point> locations;

    for (size_t i = 0; i < level_scale.size(); i++)
    {
        double scale = level_scale[i];
        Size sz(cvRound(img.cols / scale), cvRound(img.rows / scale));
        GpuMat smaller_img;

        if (sz == img.size())
            smaller_img = img;
        else
        {
            smaller_img.create(sz, img.type());
            switch (img.type()) {
                case CV_8UC1: hog::resize_8UC1(img, smaller_img); break;
                case CV_8UC4: hog::resize_8UC4(img, smaller_img); break;
            }
        }

        detect(smaller_img, locations, hit_threshold, win_stride, padding);
        Size scaled_win_size(cvRound(win_size.width * scale), cvRound(win_size.height * scale));
        for (size_t j = 0; j < locations.size(); j++)
            all_candidates.push_back(Rect(Point2d((CvPoint)locations[j]) * scale, scaled_win_size));
    }

    found_locations.assign(all_candidates.begin(), all_candidates.end());    
    groupRectangles(found_locations, group_threshold, 0.2/*magic number copied from CPU version*/);
}
#endif
#else
struct HOGCache
{
		struct BlockData
		{
				BlockData() : histOfs(0), imgOffset() {}
				int histOfs;
				Point imgOffset;
		};

		struct PixData
		{
				size_t gradOfs, qangleOfs;
				int histOfs[4];
				float histWeights[4];
				float gradWeight;
		};

		HOGCache();
		HOGCache(const HOGDescriptor* descriptor,
				const Mat& img, Size paddingTL, Size paddingBR,
				bool useCache, Size cacheStride);
		virtual ~HOGCache() {};
		virtual void init(const HOGDescriptor* descriptor,
				const Mat& img, Size paddingTL, Size paddingBR,
				bool useCache, Size cacheStride);

		Size windowsInImage(Size imageSize, Size winStride) const;
		Rect getWindow(Size imageSize, Size winStride, int idx) const;

		const float* getBlock(Point pt, float* buf);
		virtual void normalizeBlockHistogram(float* histogram) const;
		
		vector<PixData> pixData;
		vector<BlockData> blockData;

		bool useCache;
		vector<int> ymaxCached;
		Size winSize, cacheStride;
		Size nblocks, ncells;
		int blockHistogramSize;
		int count1, count2, count4;
		Point imgoffset;
		Mat_<float> blockCache;
		Mat_<uchar> blockCacheFlags;

		Mat grad, qangle;
		const HOGDescriptor* descriptor;
};

HOGCache::HOGCache()
{
		useCache = false;
		blockHistogramSize = count1 = count2 = count4 = 0;
		descriptor = 0;
}

HOGCache::HOGCache(const HOGDescriptor* _descriptor,
				const Mat& _img, Size _paddingTL, Size _paddingBR,
				bool _useCache, Size _cacheStride)
{
		init(_descriptor, _img, _paddingTL, _paddingBR, _useCache, _cacheStride);
}

void HOGCache::init(const HOGDescriptor* _descriptor,
				const Mat& _img, Size _paddingTL, Size _paddingBR,
				bool _useCache, Size _cacheStride)
{
		descriptor = _descriptor;
		cacheStride = _cacheStride;
		useCache = _useCache;

		descriptor->computeGradient(_img, grad, qangle, _paddingTL, _paddingBR);
		imgoffset = _paddingTL;

		winSize = descriptor->winSize;
		Size blockSize = descriptor->blockSize;
		Size blockStride = descriptor->blockStride;
		Size cellSize = descriptor->cellSize;
		Size winSize = descriptor->winSize;
		int i, j, nbins = descriptor->nbins;
		int rawBlockSize = blockSize.width*blockSize.height;

		nblocks = Size((winSize.width - blockSize.width)/blockStride.width + 1,
									 (winSize.height - blockSize.height)/blockStride.height + 1);
		ncells = Size(blockSize.width/cellSize.width, blockSize.height/cellSize.height);
		blockHistogramSize = ncells.width*ncells.height*nbins;

		if( useCache )
		{
				Size cacheSize((grad.cols - blockSize.width)/cacheStride.width+1,
											 (winSize.height/cacheStride.height)+1);
				blockCache.create(cacheSize.height, cacheSize.width*blockHistogramSize);
				blockCacheFlags.create(cacheSize);
				size_t i, cacheRows = blockCache.rows;
				ymaxCached.resize(cacheRows);
				for( i = 0; i < cacheRows; i++ )
						ymaxCached[i] = -1;
		}

		Mat_<float> weights(blockSize);
		float sigma = (float)descriptor->getWinSigma();
		float scale = 1.f/(sigma*sigma*2);

		for(i = 0; i < blockSize.height; i++)
				for(j = 0; j < blockSize.width; j++)
				{
						float di = i - blockSize.height*0.5f;
						float dj = j - blockSize.width*0.5f;
						weights(i,j) = std::exp(-(di*di + dj*dj)*scale);
				}

		blockData.resize(nblocks.width*nblocks.height);
		pixData.resize(rawBlockSize*3);

		// Initialize 2 lookup tables, pixData & blockData.
		// Here is why:
		//
		// The detection algorithm runs in 4 nested loops (at each pyramid layer):
		//  loop over the windows within the input image
		//    loop over the blocks within each window
		//      loop over the cells within each block
		//        loop over the pixels in each cell
		//
		// As each of the loops runs over a 2-dimensional array,
		// we could get 8(!) nested loops in total, which is very-very slow.
		//
		// To speed the things up, we do the following:
		//   1. loop over windows is unrolled in the HOGDescriptor::{compute|detect} methods;
		//         inside we compute the current search window using getWindow() method.
		//         Yes, it involves some overhead (function call + couple of divisions),
		//         but it's tiny in fact.
		//   2. loop over the blocks is also unrolled. Inside we use pre-computed blockData[j]
		//         to set up gradient and histogram pointers.
		//   3. loops over cells and pixels in each cell are merged
		//       (since there is no overlap between cells, each pixel in the block is processed once)
		//      and also unrolled. Inside we use PixData[k] to access the gradient values and
		//      update the histogram
		//
		count1 = count2 = count4 = 0;
		for( j = 0; j < blockSize.width; j++ )
				for( i = 0; i < blockSize.height; i++ )
				{
						PixData* data = 0;
						float cellX = (j+0.5f)/cellSize.width - 0.5f;
						float cellY = (i+0.5f)/cellSize.height - 0.5f;
						int icellX0 = cvFloor(cellX);
						int icellY0 = cvFloor(cellY);
						int icellX1 = icellX0 + 1, icellY1 = icellY0 + 1;
						cellX -= icellX0;
						cellY -= icellY0;
						
						if( (unsigned)icellX0 < (unsigned)ncells.width &&
								(unsigned)icellX1 < (unsigned)ncells.width )
						{
								if( (unsigned)icellY0 < (unsigned)ncells.height &&
										(unsigned)icellY1 < (unsigned)ncells.height )
								{
										data = &pixData[rawBlockSize*2 + (count4++)];
										data->histOfs[0] = (icellX0*ncells.height + icellY0)*nbins;
										data->histWeights[0] = (1.f - cellX)*(1.f - cellY);
										data->histOfs[1] = (icellX1*ncells.height + icellY0)*nbins;
										data->histWeights[1] = cellX*(1.f - cellY);
										data->histOfs[2] = (icellX0*ncells.height + icellY1)*nbins;
										data->histWeights[2] = (1.f - cellX)*cellY;
										data->histOfs[3] = (icellX1*ncells.height + icellY1)*nbins;
										data->histWeights[3] = cellX*cellY;
								}
								else
								{
										data = &pixData[rawBlockSize + (count2++)];
										if( (unsigned)icellY0 < (unsigned)ncells.height )
										{
												icellY1 = icellY0;
												cellY = 1.f - cellY;
										}
										data->histOfs[0] = (icellX0*ncells.height + icellY1)*nbins;
										data->histWeights[0] = (1.f - cellX)*cellY;
										data->histOfs[1] = (icellX1*ncells.height + icellY1)*nbins;
										data->histWeights[1] = cellX*cellY;
										data->histOfs[2] = data->histOfs[3] = 0;
										data->histWeights[2] = data->histWeights[3] = 0;
								}
						}
						else
						{
								if( (unsigned)icellX0 < (unsigned)ncells.width )
								{
										icellX1 = icellX0;
										cellX = 1.f - cellX;
								}

								if( (unsigned)icellY0 < (unsigned)ncells.height &&
										(unsigned)icellY1 < (unsigned)ncells.height )
								{
										data = &pixData[rawBlockSize + (count2++)];
										data->histOfs[0] = (icellX1*ncells.height + icellY0)*nbins;
										data->histWeights[0] = cellX*(1.f - cellY);
										data->histOfs[1] = (icellX1*ncells.height + icellY1)*nbins;
										data->histWeights[1] = cellX*cellY;
										data->histOfs[2] = data->histOfs[3] = 0;
										data->histWeights[2] = data->histWeights[3] = 0;
								}
								else
								{
										data = &pixData[count1++];
										if( (unsigned)icellY0 < (unsigned)ncells.height )
										{
												icellY1 = icellY0;
												cellY = 1.f - cellY;
										}
										data->histOfs[0] = (icellX1*ncells.height + icellY1)*nbins;
										data->histWeights[0] = cellX*cellY;
										data->histOfs[1] = data->histOfs[2] = data->histOfs[3] = 0;
										data->histWeights[1] = data->histWeights[2] = data->histWeights[3] = 0;
								}
						}
						data->gradOfs = (grad.cols*i + j)*2;
						data->qangleOfs = (qangle.cols*i + j)*2;
						data->gradWeight = weights(i,j);
				}

		assert( count1 + count2 + count4 == rawBlockSize );
		// defragment pixData
		for( j = 0; j < count2; j++ )
				pixData[j + count1] = pixData[j + rawBlockSize];
		for( j = 0; j < count4; j++ )
				pixData[j + count1 + count2] = pixData[j + rawBlockSize*2];
		count2 += count1;
		count4 += count2;

		// initialize blockData
		for( j = 0; j < nblocks.width; j++ )
				for( i = 0; i < nblocks.height; i++ )
				{
						BlockData& data = blockData[j*nblocks.height + i];
						data.histOfs = (j*nblocks.height + i)*blockHistogramSize;
						data.imgOffset = Point(j*blockStride.width,i*blockStride.height);
				}
}


const float* HOGCache::getBlock(Point pt, float* buf)
{
		float* blockHist = buf;
		assert(descriptor != 0);

		Size blockSize = descriptor->blockSize;
		pt += imgoffset;

		CV_Assert( (unsigned)pt.x <= (unsigned)(grad.cols - blockSize.width) &&
							 (unsigned)pt.y <= (unsigned)(grad.rows - blockSize.height) );
		
		if( useCache )
		{
				CV_Assert( pt.x % cacheStride.width == 0 &&
									 pt.y % cacheStride.height == 0 );
				Point cacheIdx(pt.x/cacheStride.width,
											(pt.y/cacheStride.height) % blockCache.rows);
				if( pt.y != ymaxCached[cacheIdx.y] )
				{
						Mat_<uchar> cacheRow = blockCacheFlags.row(cacheIdx.y);
						cacheRow = (uchar)0;
						ymaxCached[cacheIdx.y] = pt.y;
				}

				blockHist = &blockCache[cacheIdx.y][cacheIdx.x*blockHistogramSize];
				uchar& computedFlag = blockCacheFlags(cacheIdx.y, cacheIdx.x);
				if( computedFlag != 0 )
						return blockHist;
				computedFlag = (uchar)1; // set it at once, before actual computing
		}

		int k, C1 = count1, C2 = count2, C4 = count4;
		const float* gradPtr = (const float*)(grad.data + grad.step*pt.y) + pt.x*2;
		const uchar* qanglePtr = qangle.data + qangle.step*pt.y + pt.x*2;

		CV_Assert( blockHist != 0 );

		for( k = 0; k < blockHistogramSize; k++ )
				blockHist[k] = 0.f;
		
		const PixData* _pixData = &pixData[0];

		for( k = 0; k < C1; k++ )
		{
				const PixData& pk = _pixData[k];
				const float* a = gradPtr + pk.gradOfs;
				float w = pk.gradWeight*pk.histWeights[0];
				const uchar* h = qanglePtr + pk.qangleOfs;
				int h0 = h[0], h1 = h[1];
				float* hist = blockHist + pk.histOfs[0];
				float t0 = hist[h0] + a[0]*w;
				float t1 = hist[h1] + a[1]*w;
				hist[h0] = t0; hist[h1] = t1;
		}

		for( ; k < C2; k++ )
		{
				const PixData& pk = _pixData[k];
				const float* a = gradPtr + pk.gradOfs;
				float w, t0, t1, a0 = a[0], a1 = a[1];
				const uchar* h = qanglePtr + pk.qangleOfs;
				int h0 = h[0], h1 = h[1];
				
				float* hist = blockHist + pk.histOfs[0];
				w = pk.gradWeight*pk.histWeights[0];
				t0 = hist[h0] + a0*w;
				t1 = hist[h1] + a1*w;
				hist[h0] = t0; hist[h1] = t1;
				
				hist = blockHist + pk.histOfs[1];
				w = pk.gradWeight*pk.histWeights[1];
				t0 = hist[h0] + a0*w;
				t1 = hist[h1] + a1*w;
				hist[h0] = t0; hist[h1] = t1;
		}

		for( ; k < C4; k++ )
		{
				const PixData& pk = _pixData[k];
				const float* a = gradPtr + pk.gradOfs;
				float w, t0, t1, a0 = a[0], a1 = a[1];
				const uchar* h = qanglePtr + pk.qangleOfs;
				int h0 = h[0], h1 = h[1];
				
				float* hist = blockHist + pk.histOfs[0];
				w = pk.gradWeight*pk.histWeights[0];
				t0 = hist[h0] + a0*w;
				t1 = hist[h1] + a1*w;
				hist[h0] = t0; hist[h1] = t1;
				
				hist = blockHist + pk.histOfs[1];
				w = pk.gradWeight*pk.histWeights[1];
				t0 = hist[h0] + a0*w;
				t1 = hist[h1] + a1*w;
				hist[h0] = t0; hist[h1] = t1;

				hist = blockHist + pk.histOfs[2];
				w = pk.gradWeight*pk.histWeights[2];
				t0 = hist[h0] + a0*w;
				t1 = hist[h1] + a1*w;
				hist[h0] = t0; hist[h1] = t1;

				hist = blockHist + pk.histOfs[3];
				w = pk.gradWeight*pk.histWeights[3];
				t0 = hist[h0] + a0*w;
				t1 = hist[h1] + a1*w;
				hist[h0] = t0; hist[h1] = t1;
		}

		normalizeBlockHistogram(blockHist);

		return blockHist;
}


void HOGCache::normalizeBlockHistogram(float* _hist) const
{
		float* hist = &_hist[0];
		size_t i, sz = blockHistogramSize;
		
		float sum = 0;
		for( i = 0; i < sz; i++ )
				sum += hist[i]*hist[i];
		float scale = 1.f/(std::sqrt(sum)+sz*0.1f), thresh = (float)descriptor->L2HysThreshold;
		for( i = 0, sum = 0; i < sz; i++ )
		{
				hist[i] = std::min(hist[i]*scale, thresh);
				sum += hist[i]*hist[i];
		}
		scale = 1.f/(std::sqrt(sum)+1e-3f);
		for( i = 0; i < sz; i++ )
				hist[i] *= scale;
}
		
		
Size HOGCache::windowsInImage(Size imageSize, Size winStride) const
{
		return Size((imageSize.width - winSize.width)/winStride.width + 1,
								(imageSize.height - winSize.height)/winStride.height + 1);
}

Rect HOGCache::getWindow(Size imageSize, Size winStride, int idx) const
{
		int nwindowsX = (imageSize.width - winSize.width)/winStride.width + 1;
		int y = idx / nwindowsX;
		int x = idx - nwindowsX*y;
		return Rect( x*winStride.width, y*winStride.height, winSize.width, winSize.height );
}

struct HOGInvoker
{
		HOGInvoker( const ROIHOGDetector* _hog, const Mat& _img,
								double _hitThreshold, Size _padding,
								std::vector<DetectionROI>* locs, ConcurrentRectVector* _vec ) 
		{
				hog = _hog;
				img = _img;
				hitThreshold = _hitThreshold;
				padding = _padding;
				locations = locs;
				vec = _vec;
		}
		void operator()( const BlockedRange& range ) const
		{
				int i, i1 = range.begin(), i2 = range.end();

				Size maxSz(cvCeil(img.cols/(*locations)[0].scale), cvCeil(img.rows/(*locations)[0].scale));
				Mat smallerImgBuf(maxSz, img.type());
				vector<Point> dets;
				
				// fprintf(stderr, "start parfor\n");
				for( i = i1; i < i2; i++ )
				{
						double scale = (*locations)[i].scale;

						Size sz(cvRound(img.cols / scale), cvRound(img.rows / scale));
						Mat smallerImg(sz, img.type(), smallerImgBuf.data);

						if( sz == img.size() )
								smallerImg = Mat(sz, img.type(), img.data, img.step);
						else
								resize(img, smallerImg, sz);

#ifdef HOG_PART_SCORE
						hog->detectROI(smallerImg, (*locations)[i].locations, dets, (*locations)[i].confidences, (*locations)[i].part_score, hitThreshold, Size(), padding);
#else
						hog->detectROI(smallerImg, (*locations)[i].locations, dets, (*locations)[i].confidences, hitThreshold, Size(), padding);
#endif
						Size scaledWinSize = Size(cvRound(hog->winSize.width*scale), cvRound(hog->winSize.height*scale));
						for( size_t j = 0; j < dets.size(); j++ )
								vec->push_back(Rect(cvRound(dets[j].x*scale),
																		cvRound(dets[j].y*scale),
																		scaledWinSize.width, scaledWinSize.height));
				}
				// fprintf(stderr, "end parfor\n");
		}
		const ROIHOGDetector* hog;
		Mat img;
		double hitThreshold;
		std::vector<DetectionROI>* locations;
		Size padding;
		ConcurrentRectVector* vec;
};

#if 0
void ROIHOGDetector::detect(const Mat& img,
    vector<Point>& hits, double hitThreshold,
    Size winStride, Size padding, const vector<Point>& locations) const
{
    hits.clear();
    if( svmDetector.empty() )
        return;
    
    if( winStride == Size() )
        winStride = cellSize;
    Size cacheStride(gcd(winStride.width, blockStride.width),
                     gcd(winStride.height, blockStride.height));
    size_t nwindows = locations.size();
    padding.width = (int)alignSize(std::max(padding.width, 0), cacheStride.width);
    padding.height = (int)alignSize(std::max(padding.height, 0), cacheStride.height);
    Size paddedImgSize(img.cols + padding.width*2, img.rows + padding.height*2);
    
    HOGCache cache(this, img, padding, padding, nwindows == 0, cacheStride);

    if( !nwindows )
        nwindows = cache.windowsInImage(paddedImgSize, winStride).area();

    const HOGCache::BlockData* blockData = &cache.blockData[0];

    int nblocks = cache.nblocks.area();
    int blockHistogramSize = cache.blockHistogramSize;
    size_t dsize = getDescriptorSize();

    double rho = svmDetector.size() > dsize ? svmDetector[dsize] : 0;
    vector<float> blockHist(blockHistogramSize);

    for( size_t i = 0; i < nwindows; i++ )
    {
        Point pt0;
        if( !locations.empty() )
        {
            pt0 = locations[i];
            if( pt0.x < -padding.width || pt0.x > img.cols + padding.width - winSize.width ||
                pt0.y < -padding.height || pt0.y > img.rows + padding.height - winSize.height )
                continue;
        }
        else
        {
            pt0 = cache.getWindow(paddedImgSize, winStride, (int)i).tl() - Point(padding);
            CV_Assert(pt0.x % cacheStride.width == 0 && pt0.y % cacheStride.height == 0);
        }
        double s = rho;
        const float* svmVec = &svmDetector[0];
        int j, k;
        for( j = 0; j < nblocks; j++, svmVec += blockHistogramSize )
        {
            const HOGCache::BlockData& bj = blockData[j];
            Point pt = pt0 + bj.imgOffset;

            const float* vec = cache.getBlock(pt, &blockHist[0]);
            for( k = 0; k <= blockHistogramSize - 4; k += 4 )
                s += vec[k]*svmVec[k] + vec[k+1]*svmVec[k+1] +
                    vec[k+2]*svmVec[k+2] + vec[k+3]*svmVec[k+3];
            for( ; k < blockHistogramSize; k++ )
                s += vec[k]*svmVec[k];
        }
        if( s >= hitThreshold )
            hits.push_back(pt0);
    }
}
#endif
/**/
void ROIHOGDetector::detectROI(const cv::Mat& img, const vector<cv::Point> &locations, 
										CV_OUT std::vector<cv::Point>& foundLocations, CV_OUT std::vector<double>& confidences, 
										double hitThreshold, cv::Size winStride,
										cv::Size padding) const
{
	foundLocations.clear();
	
	confidences.clear();

	if( svmDetector.empty() )
		return;

	if( locations.empty() )
		return;

	if( winStride == Size() )
		winStride = cellSize;

	Size cacheStride(gcd(winStride.width, blockStride.width),
									 gcd(winStride.height, blockStride.height));

	size_t nwindows = locations.size();
	padding.width = (int)alignSize(std::max(padding.width, 0), cacheStride.width);
	padding.height = (int)alignSize(std::max(padding.height, 0), cacheStride.height);
	Size paddedImgSize(img.cols + padding.width*2, img.rows + padding.height*2);
	
	// HOGCache cache(this, img, padding, padding, nwindows == 0, cacheStride);
	HOGCache cache(this, img, padding, padding, true, cacheStride);
	if( !nwindows )
			nwindows = cache.windowsInImage(paddedImgSize, winStride).area();

	const HOGCache::BlockData* blockData = &cache.blockData[0];

	int nblocks = cache.nblocks.area();
	int blockHistogramSize = cache.blockHistogramSize;
	size_t dsize = getDescriptorSize();

	double rho = svmDetector.size() > dsize ? svmDetector[dsize] : 0;
	vector<float> blockHist(blockHistogramSize);

	for( size_t i = 0; i < nwindows; i++ )
	{
			Point pt0;
			pt0 = locations[i];
			if( pt0.x < -padding.width || pt0.x > img.cols + padding.width - winSize.width ||
					pt0.y < -padding.height || pt0.y > img.rows + padding.height - winSize.height )
			{
				confidences.push_back(SCORE_OUTOFIMG);
				continue;
			}

			double s = rho;
			const float* svmVec = &svmDetector[0];
			int j, k;

			for( j = 0; j < nblocks; j++, svmVec += blockHistogramSize )
			{
					const HOGCache::BlockData& bj = blockData[j];
					Point pt = pt0 + bj.imgOffset;
					// need to devide this into 4 parts!
					const float* vec = cache.getBlock(pt, &blockHist[0]);
					for( k = 0; k <= blockHistogramSize - 4; k += 4 )
							s += vec[k]*svmVec[k] + vec[k+1]*svmVec[k+1] +
									vec[k+2]*svmVec[k+2] + vec[k+3]*svmVec[k+3];
					for( ; k < blockHistogramSize; k++ )
							s += vec[k]*svmVec[k];
			}
			// cv::waitKey();
			confidences.push_back(s);

			if( s >= hitThreshold )
					foundLocations.push_back(pt0);
	}
}

#ifdef HOG_PART_SCORE
void ROIHOGDetector::detectROI(const cv::Mat& img, const vector<cv::Point> &locations, 
										CV_OUT std::vector<cv::Point>& foundLocations, CV_OUT std::vector<double>& confidences, CV_OUT std::vector<double> part_scores[], 
										double hitThreshold, cv::Size winStride,
										cv::Size padding) const
{
	foundLocations.clear();
	
	confidences.clear();

	if( svmDetector.empty() )
		return;

	if( locations.empty() )
		return;

	if( winStride == Size() )
		winStride = cellSize;

	Size cacheStride(gcd(winStride.width, blockStride.width),
									 gcd(winStride.height, blockStride.height));

	size_t nwindows = locations.size();
	padding.width = (int)alignSize(std::max(padding.width, 0), cacheStride.width);
	padding.height = (int)alignSize(std::max(padding.height, 0), cacheStride.height);
	Size paddedImgSize(img.cols + padding.width*2, img.rows + padding.height*2);
	
	// HOGCache cache(this, img, padding, padding, nwindows == 0, cacheStride);
	HOGCache cache(this, img, padding, padding, true, cacheStride);
	if( !nwindows )
			nwindows = cache.windowsInImage(paddedImgSize, winStride).area();

	const HOGCache::BlockData* blockData = &cache.blockData[0];

	int nblocks = cache.nblocks.area();
	int blockHistogramSize = cache.blockHistogramSize;
	size_t dsize = getDescriptorSize();

	double rho = svmDetector.size() > dsize ? svmDetector[dsize] : 0;
	vector<float> blockHist(blockHistogramSize);

	Point2d mp((double)(winSize.width - blockSize.width) / 2, (double)(winSize.height - blockSize.height)/ 2);
	for( size_t i = 0; i < nwindows; i++ )
	{
			Point pt0;
			pt0 = locations[i];
			if( pt0.x < -padding.width || pt0.x > img.cols + padding.width - winSize.width ||
					pt0.y < -padding.height || pt0.y > img.rows + padding.height - winSize.height )
			{
				confidences.push_back(SCORE_OUTOFIMG);
				continue;
			}

			double s = rho;
			double part_score[HOG_PART_NUM];
			memset(part_score, 0, sizeof(double) * HOG_PART_NUM);
			
			// temporary bias terms
			part_score[0] = -2.2;
			part_score[1] = -1.1;
			part_score[2] = -2.2;
			part_score[3] = -1.1;

			// int a[HOG_PART_NUM];
			// memset(a, 0, sizeof(int) * HOG_PART_NUM);
			const float* svmVec = &svmDetector[0];
			int j, k;
			for( j = 0; j < nblocks; j++, svmVec += blockHistogramSize )
			{
					const HOGCache::BlockData& bj = blockData[j];
					Point pt = pt0 + bj.imgOffset;
					// need to devide this into 4 parts!
					const float* vec = cache.getBlock(pt, &blockHist[0]);

					double temps = 0.0f;
					for( k = 0; k <= blockHistogramSize - 4; k += 4 )
							temps += vec[k]*svmVec[k] + vec[k+1]*svmVec[k+1] +
									vec[k+2]*svmVec[k+2] + vec[k+3]*svmVec[k+3];
					for( ; k < blockHistogramSize; k++ )
							temps += vec[k]*svmVec[k];
					s += temps;
					// allowing overlap
					if(bj.imgOffset.x <= mp.x)
					{
						if(bj.imgOffset.y <= mp.y) {
							part_score[0] += temps;
							// a[0]++;
						}
						if(bj.imgOffset.y >= mp.y) {
							part_score[1] += temps;
							// a[1]++;
						}
					}
					if(bj.imgOffset.x >= mp.x)
					{
						if(bj.imgOffset.y <= mp.y) {
							part_score[2] += temps;
							// a[2]++;
						}
						if(bj.imgOffset.y >= mp.y) {
							part_score[3] += temps;
							// a[3]++;
						}
					}
			}
			// std::cout << a[0] << " " << a[1] << " " << a[2] << " " << a[3] << std::endl;
			// exit(1);
			// cv::waitKey();
			confidences.push_back(s);
			for(j = 0; j < HOG_PART_NUM; j++)
				part_scores[j].push_back(part_score[j]);

			if( s >= hitThreshold )
					foundLocations.push_back(pt0);
	}
}
#endif
void ROIHOGDetector::detectMultiScaleROI(const cv::Mat& img, 
															CV_OUT std::vector<cv::Rect>& foundLocations,
															std::vector<DetectionROI>& locations,
															double hitThreshold,
															int groupThreshold) const
{
	ConcurrentRectVector allCandidates;

	parallel_for(BlockedRange(0, (int)locations.size()),
						 HOGInvoker(this, img, hitThreshold, Size(8, 8), &locations, &allCandidates));
		
	foundLocations.resize(allCandidates.size());
	std::copy(allCandidates.begin(), allCandidates.end(), foundLocations.begin());
	cv::groupRectangles(foundLocations, groupThreshold, 0.2);
}

void ROIHOGDetector::drawOneCell(cv::Mat &img, const float* w, const float minval, const float maxval)
{
	for(int i = 0; i < nbins; i++) 
	{
		float rad = M_PI / nbins * i + M_PI / 2;
		cv::Point2d pt1(cos(rad), sin(rad));
		cv::Point2d pt2(-cos(rad), -sin(rad));

		pt1.x = (pt1.x + 1) * img.cols/2;
		pt1.y = (pt1.y + 1) * img.rows/2;
		pt2.x = (pt2.x + 1) * img.cols/2;
		pt2.y = (pt2.y + 1) * img.rows/2;

		unsigned char weight = (unsigned char)(((w[i] - minval) / (maxval - minval)) * 255.0);

		cv::line(img, pt1, pt2, cv::Scalar(weight));
	}
}

void ROIHOGDetector::tempCropUpperBodyDetector()
{
	cv::Size newWinSize(winSize.width, winSize.height / 2);
	// cv::Size newWinSize(winSize.width, 48);
	// crop svm model...
	// num blocks
	cv::Size numBlocks((winSize.width - blockSize.width) / blockStride.width + 1, 
											(winSize.height - blockSize.height) / blockStride.height + 1);
	cv::Size newNumBlocks((newWinSize.width - blockSize.width) / blockStride.width + 1, 
											(newWinSize.height - blockSize.height) / blockStride.height + 1);
	// block size
	cv::Size numCells(blockSize.width / cellSize.width, blockSize.height / cellSize.height);

	printf("winSize %d, %d\n", winSize.width, winSize.height);
	printf("newWinSize %d, %d\n", newWinSize.width, newWinSize.height);
	printf("numBlocks %d, %d\n", numBlocks.width, numBlocks.height);
	printf("numNewBlocks %d, %d\n", newNumBlocks.width, newNumBlocks.height);
	printf("numCells %d, %d\n", numCells.width, numCells.height);

	vector<float> newDetector;
	int idx = 0;

	for(int col = 0; col < numBlocks.width; col++)
	{
		for(int i = 0; i < newNumBlocks.height * numCells.width * numCells.height * nbins; i++)
		{
			newDetector.push_back(svmDetector[i + idx]);
		}
		idx += numBlocks.height * numCells.width * numCells.height * nbins;
	}
	newDetector.push_back(svmDetector[idx]);
	svmDetector.clear();
	svmDetector = newDetector;
	winSize = newWinSize;
}

void ROIHOGDetector::showModel()
{
	int cellImgSize = 40;
	cv::Mat img(((winSize.height - blockSize.height) / blockStride.height + 1)
									* (blockSize.height / cellSize.height) * cellImgSize
						, ((winSize.width - blockSize.width) / blockStride.width + 1)
									* (blockSize.width / cellSize.width) * cellImgSize
						, CV_8U, cv::Scalar(0));

	const float* svmVec = &svmDetector[0];

	const float maxval = *std::max_element( svmDetector.begin(), svmDetector.end() - 1);
	const float minval = *std::min_element( svmDetector.begin(), svmDetector.end() - 1);

	for(int j = 0; j < ((winSize.width - blockSize.width) / blockStride.width + 1); j++) {
		for(int i = 0; i < ((winSize.height - blockSize.height) / blockStride.height + 1); i++) {
			for(int bx = 0; bx < blockSize.width / cellSize.width; bx++) {
				for(int by = 0; by < blockSize.height / cellSize.height; by++) {
					cv::Mat patch(img, cv::Rect(j * cellImgSize * 2 + bx * cellImgSize
												, i * cellImgSize * 2 + by * cellImgSize
												, cellImgSize, cellImgSize));
					drawOneCell(patch, svmVec, minval, maxval);
					svmVec += nbins; 
				}
			}
		}
	}
	static int counter = 0;

	ostringstream winname;
	winname << "HOG" << counter++;
	cv::imshow(winname.str(), img);
	cv::waitKey();
}

#endif
// 
void ROIHOGDetector::readALTModel(std::string modelfile)
{
	// read model from SVMlight format..
	FILE *modelfl;
	if ((modelfl = fopen(modelfile.c_str(), "rb")) == NULL)
	{ 
		std::string eerr("file not exist");
		std::string efile(__FILE__);
		std::string efunc(__FUNCTION__);
		throw Exception(CV_StsError, eerr, efile, efunc, __LINE__); 
	}
	char version_buffer[10];
	if (!fread (&version_buffer,sizeof(char),10,modelfl))
	{ 
		std::string eerr("version?");
		std::string efile(__FILE__);
		std::string efunc(__FUNCTION__);
		throw Exception(CV_StsError, eerr, efile, efunc, __LINE__); 
	}
	if(strcmp(version_buffer,"V6.01")) {
		std::string eerr("version doesnot match");
		std::string efile(__FILE__);
		std::string efunc(__FUNCTION__);
		throw Exception(CV_StsError, eerr, efile, efunc, __LINE__); 
	}
	/* read version number */
	int version = 0;
	if (!fread (&version,sizeof(int),1,modelfl))
	{ throw Exception(); }
	if (version < 200)
	{ 
		std::string eerr("version doesnot match");
		std::string efile(__FILE__);
		std::string efunc(__FUNCTION__);
		throw Exception(); 
	}
	int kernel_type;
	int nread;
	nread=fread(&(kernel_type),sizeof(int),1,modelfl);

	{// ignore these
		int poly_degree;
		nread=fread(&(poly_degree),sizeof(int),1,modelfl);

		double rbf_gamma;
		nread=fread(&(rbf_gamma),sizeof(double), 1, modelfl);
		double coef_lin;
		nread=fread(&(coef_lin),sizeof(double),1,modelfl); 
		double coef_const;
		nread=fread(&(coef_const),sizeof(double),1,modelfl);
		int l;
		nread=fread(&l,sizeof(int),1,modelfl);
		char* custom = new char[l];
		nread=fread(custom,sizeof(char),l,modelfl);
		delete[] custom;
	}
	int totwords;
	nread=fread(&(totwords),sizeof(int),1,modelfl);
	{// ignore these
		int totdoc;
		nread=fread(&(totdoc),sizeof(int),1,modelfl);
		int sv_num;
		nread=fread(&(sv_num), sizeof(int),1,modelfl);
	}

	double linearbias;
	nread=fread(&linearbias, sizeof(double), 1, modelfl);

	std::vector<float> detector;
	detector.clear();
	if(kernel_type == 0) { /* linear kernel */
		/* save linear wts also */
		double *linearwt = new double[totwords+1];
		int length = totwords;
		nread = fread(linearwt, sizeof(double), totwords + 1, modelfl);
		if(nread != length + 1)
			throw Exception();
		/*
		for(int i = 0; i < 10; i++)
			printf("%.02f, ", svmDetector[i]);
		printf("... %.02f [%d]\n", svmDetector[svmDetector.size()-1], svmDetector.size());
		*/
#if 1
		for(int i = 0; i < length; i++)
			detector.push_back((float)linearwt[i]);
		detector.push_back((float)-linearbias);
		setSVMDetector(detector);
#else
		svmDetector.clear();
		for(int i = 0; i < length; i++)
			svmDetector.push_back((float)linearwt[i]);
		svmDetector.push_back((float)-linearbias);
#endif
#if 0
		for(int i = 0; i < 10; i++)
			printf("%.02f, ", detector[i]);
		printf("... %.02f [%d]\n", detector[detector.size()-1], detector.size());
#endif
		// showModel();
		delete linearwt;
	} else {
		throw Exception();
	}
	fclose(modelfl);
}
/* == HOG modification == */



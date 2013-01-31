#pragma once

#include <string>
#include <vector>
#include "gameplay.h"

#define DEFAULT_E_SIGMA 200.0
#define DEFAULT_BETA  400.0
#define DEFAULT_LAMBDA  0.4
#define DEFAULT_KAPPA  1.0
#define MAX_RESAMPLING_PTS 1000

using namespace gameplay;

typedef std::vector<Vector2> GesturePoints;

class GestureTemplate {
	friend class GestureRecognizer;
public:
	GestureTemplate(const GestureTemplate& other);
	GestureTemplate(const std::string &id, const std::vector<Vector2>& points);
	GesturePoints _points;
protected:
	std::string _id;
};

class GestureResult {
protected:
	GestureTemplate _template;
	double _probability;
	
public:
	bool operator == (const GestureResult& result) const;
	GestureResult(const GestureResult& other);
	GestureResult(const GestureTemplate& temp, double prob);
};

class GestureRecognizer {
public:
	GestureRecognizer();
	GestureRecognizer(const GesturePoints& points, int samplePointDistance = 5);
	void setTemplates(const std::vector<GestureTemplate>& templates);
	std::list<GestureResult> recognize(const GesturePoints &points,
		double beta = DEFAULT_BETA,
		double lambda = DEFAULT_LAMBDA,
		double kappa = DEFAULT_KAPPA,
		double e_sigma = 200.0);

	/**
	 * Normalizes a point sequence so that it is scaled and centred within a defined box.
	 * 
	 * (This method was implemented and exposed in the public interface to ease the
	 * implementation of the demonstrator. This method is not used by the recognition
	 * algorithm.) 
	 * 
	 * @param pts an input point sequence
	 * @param x the horizontal component of the upper-left corner of the defined box
	 * @param y the vertical component of the upper-left corner of the defined box
	 * @param width the width of the defined box
	 * @param height the height of the defined box
	 * @return a newly created point sequence that is centred and fits within the defined box 
	 */
	std::vector<Vector2> normalize(const GesturePoints& points, int x, int y, int width, int height);


private:
	
	class Pattern {
	public:
		GestureTemplate gtemplate;
		std::list<GesturePoints> segments;
		
		Pattern(const Pattern& other);
		Pattern(const GestureTemplate &temp, const std::list<GesturePoints>& segm);
	};
	typedef std::list<Pattern*> GesturePatterns;

	class Centroid {
	public:
		double x;
		double y;
		Centroid(double x, double y);
	};

	class IncrementalResult {
	public:
		const Pattern *pattern;
		double prob;
		int indexOfMostLikelySegment;

		IncrementalResult(const Pattern *pattern, double prob, int indexOfMostLikelySegment);
	};



	Rectangle normalizedSpace;
	GesturePatterns patterns;
	int samplePointDistance;


	std::vector<GestureResult*> getResults(const std::vector<IncrementalResult*>& incremental);
	std::list<GestureRecognizer::IncrementalResult*> GestureRecognizer::getIncrementalResults(const GesturePoints& input, 
	double beta, double lambda, double kappa, double e_sigma);
	void normalize(GesturePoints *pts);
	void scaleTo(GesturePoints *pts, const Rectangle& targetBounds);
	void scale(GesturePoints *pts, double sx, double sy, double originX, double originY);
	void scale(GesturePoints *pts, double sx, double sy);
	void translate(GesturePoints *pts, double dx, double dy);
	Centroid *getCentroid(const GesturePoints *pts);
	Rectangle getBoundingBox(const GesturePoints *pts);
	void marginalizeIncrementalResults(std::list<GestureRecognizer::IncrementalResult*>& results);
	std::list<GestureResult> getResults(const std::list<GestureRecognizer::IncrementalResult*>& incrementalResults);

	void clearPatterns();

	GestureRecognizer::IncrementalResult *getIncrementalResult(const GesturePoints& unkPts, 
	const Pattern *pattern, double beta, double lambda, double e_sigma);
	GesturePoints resample(const GesturePoints& points, int numTargetPoints);
	int *pointsToArray(const GesturePoints& points);
	int resample(int *template_points, int *buffer, int n, int numTargetPoints);
	double getSpatialLength(const GesturePoints& pts);
	int getSpatialLength(const int* pat, int n);
	int distance(int x1, int y1, int x2, int y2);
	double getSegmentPoints(const int* pts, int n, double length, int* buffer);
	std::list<GesturePoints> generateEquiDistantProgressiveSubSequences(const GesturePoints& pts, int ptSpacing);
	int getResamplingPointCount(const GesturePoints& pts, int samplePointDistance);

	double getLikelihoodOfMatch(const GesturePoints& pts1, const GesturePoints& pts2,
		double eSigma, double aSigma, double lambda);
	double getEuclidianDistance(const GesturePoints& pts1, const GesturePoints& pts2);
	double getTurningAngleDistance(const GesturePoints& pts1, const GesturePoints& pts2);
	double getEuclideanDistance(const Vector2& pt1, const Vector2& pt2);
	double getTurningAngleDistance(const Vector2& ptA1, const Vector2& ptA2, const Vector2& ptB1, const Vector2& ptB2);

};
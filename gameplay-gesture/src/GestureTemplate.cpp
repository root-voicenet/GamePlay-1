#include "GestureTemplate.h"
#include <cmath>


GestureRecognizer::Centroid::Centroid(double x, double y)
{
	this->x = x;
	this->y = y;
}

GestureRecognizer::Pattern::Pattern(const Pattern &other)
	: gtemplate(other.gtemplate), segments(other.segments)
{
}

GestureRecognizer::Pattern::Pattern(const GestureTemplate &temp, const std::list<GesturePoints>& segm)
: gtemplate(temp), segments(segm)
{
	
}

GestureRecognizer::IncrementalResult::IncrementalResult(const Pattern *pat, double p, int index)
	: pattern(pat), prob(p),indexOfMostLikelySegment(index)
{
}

GestureRecognizer::GestureRecognizer()
	: normalizedSpace(0, 0, 1000, 1000), samplePointDistance(200)
{

}

std::list<GestureResult> GestureRecognizer::recognize(const GesturePoints &points, double beta, double lambda, double kappa, double e_sigma)
{
	if (points.size() < 2) {
			throw new std::exception("input must consist of at least two points");
	}
	
	std::list<IncrementalResult*> incResults = getIncrementalResults(points, beta, lambda, kappa, e_sigma);
	std::list<GestureResult> results = getResults(incResults);
	
	for(std::list<IncrementalResult*>::iterator it = incResults.begin(); it != incResults.end(); ++it)
		delete (*it);

	return results;
}

void GestureRecognizer::setTemplates(const std::vector<GestureTemplate>& templates)
{
	clearPatterns();
	for(std::vector<GestureTemplate>::const_iterator tt = templates.begin(); tt != templates.end(); ++tt) {
		GestureTemplate t(*tt);
		normalize(&t._points);
		patterns.push_back(new Pattern(t, generateEquiDistantProgressiveSubSequences(t._points, 200)));
	}

	for(GesturePatterns::iterator p = patterns.begin(); p != patterns.end(); ++p) {
		std::list<GesturePoints> segments;
		for(std::list<GesturePoints>::iterator s = (*p)->segments.begin(); s != (*p)->segments.end(); ++s) {
			GesturePoints newPts = (*s);
			normalize(&newPts);
			segments.push_back(resample(newPts, getResamplingPointCount(newPts, samplePointDistance)));
		}
		(*p)->segments = segments;
	}
}

void GestureRecognizer::clearPatterns() 
{
	for(GesturePatterns::iterator it = patterns.begin(); it != patterns.end(); ++it) {
		delete (*it);
	}

	patterns.clear();
}

std::list<GestureRecognizer::IncrementalResult*> GestureRecognizer::getIncrementalResults(const GesturePoints& input, 
	double beta, double lambda, double kappa, double e_sigma)
{
	std::list<GestureRecognizer::IncrementalResult*> res;
	GesturePoints unkPts = input;
	normalize(&unkPts);

	for(GesturePatterns::iterator it = patterns.begin(); it != patterns.end(); ++it) {
		IncrementalResult *result = getIncrementalResult(unkPts, *it, beta, lambda, e_sigma);
		GesturePoints lastSegmentPts = (*it)->segments.back();
		double completeProb = getLikelihoodOfMatch(resample(unkPts, lastSegmentPts.size()), lastSegmentPts, e_sigma, e_sigma/beta, lambda);
		double x = 1 - completeProb;
		result->prob *= (1 + kappa*exp(-x*x));
		res.push_back(result);
	}

	marginalizeIncrementalResults(res);
	return res;
}

 void GestureRecognizer::marginalizeIncrementalResults(std::list<GestureRecognizer::IncrementalResult*>& results)
 {
		double totalMass = 0.0;
		for(std::list<IncrementalResult*>::iterator it = results.begin(); it != results.end(); ++it) {
			totalMass += (*it)->prob; 
		}
		for(std::list<IncrementalResult*>::iterator it = results.begin(); it != results.end(); ++it) {
			(*it)->prob /= totalMass;
		}
}

 std::list<GestureResult> GestureRecognizer::getResults(const std::list<GestureRecognizer::IncrementalResult*>& incrementalResults) 
 {
		std::list<GestureResult> results;
		for(std::list<IncrementalResult*>::const_iterator it = incrementalResults.begin(); it != incrementalResults.end(); ++it) {
			const GestureTemplate &temp = (*it)->pattern->gtemplate;
			results.push_back(GestureResult(temp, (*it)->prob));
		}
		return results;
}

GestureRecognizer::IncrementalResult *GestureRecognizer::getIncrementalResult(const GesturePoints& unkPts, 
	const Pattern *pattern, double beta, double lambda, double e_sigma)
{
		std::list<GesturePoints> segments = pattern->segments;
		double maxProb = 0.0;
		int maxIndex = -1, i = 0;
		for(std::list<GesturePoints>::const_iterator it = segments.begin(); it != segments.end(); ++it, ++i) {
			const GesturePoints& pts = (*it);
			int samplingPtCount = pts.size();
			GesturePoints unkResampledPts = resample(unkPts, samplingPtCount);
			double prob = getLikelihoodOfMatch(unkResampledPts, pts, e_sigma, e_sigma/beta, lambda);
			if (prob > maxProb) {
				maxProb = prob;
				maxIndex = i;
			}
		}
		return new IncrementalResult(pattern, maxProb, maxIndex);
}

GesturePoints GestureRecognizer::resample(const GesturePoints& points, int numTargetPoints)
{
		GesturePoints r;
		int* inArray = pointsToArray(points);
		int* outArray = new int[numTargetPoints * 2];
		
		int outLength = resample(inArray, outArray, points.size(), numTargetPoints);
		for (int i = 0, n = outLength; i < n; i+= 2) {
			r.push_back(Vector2(outArray[i], outArray[i + 1]));
		}
		
		delete[] inArray;
		delete[] outArray;
		return r;
}

int GestureRecognizer::resample(int *template_points, int *buffer, int n, int numTargetPoints) 
{
		int *segment_buf = new int[MAX_RESAMPLING_PTS];

		double l, segmentLen, horizRest, verticRest, dx, dy;
		int x1, y1, x2, y2;
		int i, m, a, segmentPoints, j, maxOutputs, end;

		m = n * 2;
		l = getSpatialLength(template_points, n);
		segmentLen = l / (numTargetPoints - 1);
		getSegmentPoints(template_points, n, segmentLen, segment_buf);
		horizRest = 0.0f;
		verticRest = 0.0f;
		x1 = template_points[0];
		y1 = template_points[1];
		a = 0;
		maxOutputs = numTargetPoints * 2;
		for (i = 2; i < m; i += 2) {
			x2 = template_points[i];
			y2 = template_points[i + 1];
			segmentPoints = segment_buf[(i / 2) - 1];
			dx = -1.0f;
			dy = -1.0f;
			if (segmentPoints - 1 <= 0) {
				dx = 0.0f;
				dy = 0.0f;
			}
			else {
				dx = (x2 - x1) / (double) (segmentPoints);
				dy = (y2 - y1) / (double) (segmentPoints);
			}
			if (segmentPoints > 0) {
				for (j = 0; j < segmentPoints; j++) {
					if (j == 0) {
						if (a < maxOutputs) {
							buffer[a] = (int) (x1 + horizRest);
							buffer[a + 1] = (int) (y1 + verticRest);
							horizRest = 0.0;
							verticRest = 0.0;
							a += 2;
						}
					}
					else {
						if (a < maxOutputs) {
							buffer[a] = (int) (x1 + j * dx);
							buffer[a + 1] = (int) (y1 + j * dy);
							a += 2;
						}
					}
				}
			}
			x1 = x2;
			y1 = y2;
		}
		end = (numTargetPoints * 2) - 2;
		if (a < end) {
			for (i = a; i < end; i += 2) {
				buffer[i] = (buffer[i - 2] + template_points[m - 2]) / 2;
				buffer[i + 1] = (buffer[i - 1] + template_points[m - 1]) / 2;
			}
		}
		buffer[maxOutputs - 2] = template_points[m - 2];
		buffer[maxOutputs - 1] = template_points[m - 1];
		delete[] segment_buf;

		return maxOutputs;
}

double GestureRecognizer::getSpatialLength(const GesturePoints& pts) 
{
		double len = 0.0;
		GesturePoints::const_iterator i = pts.begin();
		if (i != pts.end()) {
			Vector2 p0 = *i; i++;
			while (i != pts.end()) {
				Vector2 p1 = *i; ++i;
				len += p0.distance(p1);
				p0 = p1;
			}
		}
		return len;
}

double GestureRecognizer::getSegmentPoints(const int* pts, int n, double length, int* buffer)
{
		int i, m;
		int x1, y1, x2, y2, ps;
		double rest, currentLen;

		m = n * 2;
		rest = 0.0f;
		x1 = pts[0];
		y1 = pts[1];
		for (i = 2; i < m; i += 2) {
			x2 = pts[i];
			y2 = pts[i + 1];
			currentLen = distance(x1, y1, x2, y2);
			currentLen += rest;
			rest = 0.0f;
			ps = (int) ((currentLen / length));
			if (ps == 0) {
				rest += currentLen;
			}
			else {
				rest += currentLen - (ps * length);
			}
			if (i == 2 && ps == 0) {
				ps = 1;
			}
			buffer[(i / 2) - 1] = ps;
			x1 = x2;
			y1 = y2;
		}
		return rest;
}

double GestureRecognizer::getLikelihoodOfMatch(const GesturePoints& pts1, const GesturePoints& pts2,
	double eSigma, double aSigma, double lambda) 
{
	if (eSigma == 0 || eSigma < 0) {
		throw new std::exception("eSigma must be positive");
	}
	if (aSigma == 0 || eSigma < 0) {
		throw new std::exception("aSigma must be positive");
	}
	if (lambda < 0 || lambda > 1) {
		throw new std::exception("lambda must be in the range between zero and one");
	}
	double x_e = getEuclidianDistance(pts1, pts2);
	double x_a = getTurningAngleDistance(pts1, pts2);
	return exp(- (x_e * x_e / (eSigma * eSigma) * lambda + x_a * x_a / (aSigma * aSigma) * (1 - lambda)));
}

double GestureRecognizer::getEuclidianDistance(const GesturePoints& pts1, const GesturePoints& pts2) 
{
	if (pts1.size() != pts2.size()) {
		throw new std::exception("lists must be of equal lengths");
	}
	int n = pts1.size();
	double td = 0;
	for (int i = 0; i < n; i++) {
		td += getEuclideanDistance(pts1[i], pts2[i]);
	}
	return td / n;
}

double GestureRecognizer::getTurningAngleDistance(const GesturePoints& pts1, const GesturePoints& pts2) 
{
	if (pts1.size() != pts2.size()) {
		throw new std::exception("lists must be of equal lengths");
	}
	int n = pts1.size();
	double td = 0;
	for (int i = 0; i < n - 1; i++) {
		td+= abs(getTurningAngleDistance(pts1[i], pts1[i + 1], pts2[i], pts2[i + 1]));
	}

	return td / (n - 1);
}

double GestureRecognizer::getEuclideanDistance(const Vector2& pt1, const Vector2& pt2) 
{
	return sqrt(pt1.distance(pt2));
}
	
double GestureRecognizer::getTurningAngleDistance(const Vector2& ptA1, const Vector2& ptA2, const Vector2& ptB1, const Vector2& ptB2)
{		
	double len_a = getEuclideanDistance(ptA1, ptA2);
	double len_b = getEuclideanDistance(ptB1, ptB2);
	if (len_a == 0 || len_b == 0) {
		return 0.0;
	}
	else {
		float cos = (float)(((ptA1.x - ptA2.x) * (ptB1.x - ptB2.x) + (ptA1.y - ptA2.y)*(ptB1.y - ptB2.y) ) / (len_a * len_b));
		if (abs(cos) > 1.0) {
			return 0.0;
		}
		else {
			return acos(cos);
		}
	}
}

int GestureRecognizer::getSpatialLength(const int* pat, int n) 
{
	int l;
	int i, m;
	int x1, y1, x2, y2;

	l = 0;
	m = 2 * n;
	if (m > 2) {
		x1 = pat[0];
		y1 = pat[1];
		for (i = 2; i < m; i += 2) {
			x2 = pat[i];
			y2 = pat[i + 1];
			l += distance(x1, y1, x2, y2);
			x1 = x2;
			y1 = y2;
		}
		return l;
	}
	else {
		return 0;
	}
}

int GestureRecognizer::getResamplingPointCount(const GesturePoints& pts, int samplePointDistance) 
{
		double len = getSpatialLength(pts);
		return (int)(len / samplePointDistance) + 1;
}

std::list<GesturePoints> GestureRecognizer::generateEquiDistantProgressiveSubSequences(const GesturePoints& pts, int ptSpacing)
{
		std::list<GesturePoints> sequences;
		int nSamplePoints = getResamplingPointCount(pts, ptSpacing);
		GesturePoints resampledPts = resample(pts, nSamplePoints);
		for (int i = 1, n = resampledPts.size(); i < n; i++) {
			GesturePoints seq = GesturePoints(resampledPts.begin(), resampledPts.begin() + i + 1);
			sequences.push_back(seq);
		}
		return sequences;
	}

void GestureRecognizer::normalize(GesturePoints *pts)
{
		scaleTo(pts, normalizedSpace);
		Centroid *c = getCentroid(pts);
		translate(pts, -c->x, -c->y);
		delete c;
}

int GestureRecognizer::distance(int x1, int y1, int x2, int y2)
{
	if ((x2 -= x1) < 0) {
		x2 = -x2;
	}
	if ((y2 -= y1) < 0) {
		y2 = -y2;
	}
	return (x2 + y2 - (((x2 > y2) ? y2 : x2) >> 1) );
}
	
void GestureRecognizer::scaleTo(GesturePoints *pts, const Rectangle& targetBounds)
{
	Rectangle bounds = getBoundingBox(pts);
	double a1 = (double)(targetBounds.width);
	double a2 = (double)(targetBounds.height);
	double b1 = (double)(bounds.width);
	double b2 = (double)(bounds.height);
	double s = sqrt(a1 * a1 + a2 * a2) / sqrt(b1 * b1 + b2 * b2);
	
	scale(pts, s, s, bounds.x, bounds.y);
}
	
void GestureRecognizer::scale(GesturePoints *pts, double sx, double sy, double originX, double originY) 
{
		translate(pts, -originX, -originY);
		scale(pts, sx, sy);
		translate(pts, originX, originY);
}
	
void GestureRecognizer::scale(GesturePoints *pts, double sx, double sy) 
{
	for (GesturePoints::iterator it = pts->begin(); it != pts->end(); ++it) {
		(it)->x *= sx;
		(it)->y *= sy;
	}
}
	
 void GestureRecognizer::translate(GesturePoints *pts, double dx, double dy) 
 {
	 for (GesturePoints::iterator it = pts->begin(); it != pts->end(); ++it) {
		 (it)->x += floor(dx);
		 (it)->y += floor(dy);
	 }
}

GestureRecognizer::Centroid *GestureRecognizer::getCentroid(const GesturePoints *pts) 
{
		double totalMass = pts->size();
		double xIntegral = 0.0;
		double yIntegral = 0.0;
		for (GesturePoints::const_iterator it = pts->begin(); it != pts->end(); ++it) {
			xIntegral+= it->x;
			yIntegral+= it->y;
		}
		return new Centroid(xIntegral / totalMass, yIntegral / totalMass);
}

Rectangle GestureRecognizer::getBoundingBox(const GesturePoints *pts) 
{
	int minX = INT_MAX;
	int minY = INT_MAX;
	int maxX = INT_MIN;
	int maxY = INT_MIN;

	for (GesturePoints::const_iterator it = pts->begin(); it != pts->end(); ++it) {
		int x = it->x;
		int y = it->y;
		if (x < minX) {
			minX = x;
		}
		if (x > maxX) {
			maxX = x;
		}
		if (y < minY) {
			minY = y;
		}
		if (y > maxY) {
			maxY = y;
		}
	}
	return Rectangle(minX, minY, (maxX - minX), (maxY - minY));
}

int *GestureRecognizer::pointsToArray(const GesturePoints& points) 
{
	int* out = new int[points.size() * 2];
	for (int i = 0, n = points.size() * 2; i < n; i+= 2) {
		out[i] = (int)points[i / 2].x;
		out[i + 1] = (int)points[i / 2].y;
	}
	return out;
}


GestureTemplate::GestureTemplate(const GestureTemplate& other) :
	_id(other._id),_points(other._points)
{
}

GestureTemplate::GestureTemplate(const std::string &id, const std::vector<Vector2>& points)
	: _id(id), _points(points)
{

}

GestureResult::GestureResult(const GestureResult& other)
	: _template(other._template), _probability(other._probability)
{
}

GestureResult::GestureResult(const GestureTemplate& temp, double prob)
	: _template(temp), _probability(prob)
{

}
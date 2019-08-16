from __future__ import print_function

from .miscUtil import prettyString, makeDiag, argmaxWithVal
from . import miscUtil

from functools import reduce
from itertools import product
from numpy import exp, linalg, complex128, identity, mat, prod, zeros, mod
import random
import operator as op
import copy
import scipy.stats
import scipy.special as ss
import scipy.stats as stats
import operator
import numpy as np
import math


# Think of this as a mixture of a distribution and something like a uniform,
# but it doesn't really work unless the universe is finite
class HedgedDist(object):
    def __init__(self, baseDist, p, universe=None):
        self.baseDist = baseDist
        self.p = p
        self.n = len(universe) if universe is not None else 0

    def mode(self):
        return self.baseDist.mode()

    def prob(self, v):
        return self.baseDist.prob(v) * self.p + \
               (1.0 / self.n) if self.n > 0 else 0


class Distribution(object):

    def prob(self, x):
        raise NotImplementedError()

    def draw(self):
        raise NotImplementedError()

    def sample(self):
        return self.draw()


class DiscreteDist(Distribution):
    """
    Probability distribution over a discrete domain.  This is a
    generic superclass;  do not instantiate it.  Any subclass has to
    provide methods: C{prob}, C{setProb}, C{support}
    """

    def setProb(self, x, p):
        raise NotImplementedError()

    def support(self):
        raise NotImplementedError()

    def draw(self):
        """
        @returns: a randomly drawn element from the distribution
        """
        # TODO: numpy.random.choice
        r = random.random()
        total = 0.0
        for v in self.support():
            total += self.prob(v)
            if r < total:
                return v
        raise Exception('Failed to draw from:' + str(self))

    def expectation(self, vals):
        return sum(self.prob(x) * vals[x] for x in self.support())

    def expectationF(self, f):
        return sum(self.prob(x) * f(x) for x in self.support())

    def maxProbElt(self):
        """
        @returns: The element in this domain with maximum probability
        """
        return argmaxWithVal(self.support(), self.prob)

    # Returns the x with highest weight
    def mode(self):
        return self.maxProbElt()[0]

    # Estimate the mean.  Assumes that + and * are defined on the elements
    def mean(self):
        return sum(x * self.prob(x) for x in self.support())

    # Estimate the variance.  Assumes we can do x * x, and then scale
    # and sum the results.
    def variance(self):
        mu = self.mean()
        return sum((x - mu) ** 2 * self.prob(x) for x in self.support())

    def conditionOnVar(self, index, value):
        """
        @param index: index of a variable in the joint distribution
        @param value: value of that variable

        @returns: new distribution, conditioned on variable C{i}
        having value C{value}, and with variable C{i} removed from all
        of the elements (it's redundant at this point).
        """
        newElements = [elt for elt in self.support() if elt[index] == value]
        z = sum(self.prob(elt) for elt in newElements)
        return DDist({removeElt(elt, index): self.prob(elt) / z
                      for elt in newElements})

    def verify(self, epsilon=1e-5):
        z = sum(self.prob(elt) for elt in self.support())
        assert (1 - epsilon) < z < (1 + epsilon), 'degenerate distribution ' + str(self)


def convertToDDist(oldD):
    newD = DDist({})
    for elt in oldD.support():
        newD.addProb(elt, oldD.prob(elt))
    return newD


class DDist(DiscreteDist):
    """Discrete distribution represented as a dictionary.  Can be
    sparse, in the sense that elements that are not explicitly
    contained in the dictionary are assuemd to have zero probability."""

    def __init__(self, dictionary={}, name=None):
        self.__d = copy.copy(dictionary)
        """ Dictionary whose keys are elements of the domain and values
        are their probabilities. """
        self.name = name
        """Optional name;  used in sum-out operations"""
        self.normalize()

    def copy(self):
        return DDist(self.__d, self.name)

    def update(self):
        pass

    def computeMPE(self):
        # returns pair: (element, prob)
        return max(self.__d.items(), key=lambda pair: pair[1])

    maxProbElt = computeMPE

    def mode(self):
        # just the element
        element, prob = self.maxProbElt()
        return element

    def addProb(self, val, p):
        """
        Increase the probability of element C{val} by C{p}
        """
        self.setProb(val, self.prob(val) + p)

    def mulProb(self, val, p):
        """
        Multiply the probability of element C{val} by C{p}
        """
        self.setProb(val, self.prob(val) * p)

    def prob(self, elt):
        """
        @returns: the probability associated with C{elt}
        """
        return self.__d.get(elt, 0.0)

    def setProb(self, elt, p):
        """
        @param elt: element of the domain
        @param p: probability
        Sets probability of C{elt} to be C{p}
        """
        self.__d[elt] = p

    def support(self):
        """
        @returns: A list (in any order) of the elements of this
        distribution with non-zero probability.
        """
        return [x for x in self.__d.keys() if self.__d[x] > 0.0]

    def normalize(self, z=None):
        """
        Divides all probabilities through by the sum of the values to
        ensure the distribution is normalized.

        Changes the distribution!!  (And returns it, for good measure)

        Generates an error if the sum of the current probability
        values is zero.
        """
        if not z:
            z = sum(self.prob(e) for e in self.support())
        assert z > 0.0, 'degenerate distribution ' + str(self)
        alpha = 1.0 / z
        newD = {}
        for e in self.support():
            newD[e] = self.__d[e] * alpha
        self.__d = newD
        return self

    def normalizeOrSmooth(self):
        """
        If total prob is < 1, then spread that mass out uniformly
        (among elements with non-zero probabilty, seince we don't
        really know the universe; otherwiese, just normalize.
        """
        z = sum(self.prob(e) for e in self.support())
        assert z > 0.0, 'degenerate distribution ' + str(self)
        newD = DDist({})
        if z < 1:
            beta = (1 - z) / len(self.support())
            for e in self.support():
                newD.addProb(e, beta)
        return self.normalize()

    def smooth(self, totalP):
        # redistribute p of the probability mass to the values that have
        # positive support
        n = len(self.support())
        p = totalP / n
        for e in self.support():
            self.addProb(e, p)
        self.normalize()

    def blur(self, blurDomain, blurFactor):
        """
        Adds small prob to all elements in C{blurDomain} and
        renormalizes.  Side effects the distribution.
        @param blurDomain: set of elements to be blurred
        @param blurFactor: how much to blur; 1 obliterates everything,
                           0 has no effect
        """
        eps = blurFactor * (1.0 / len(blurDomain))
        for elt in blurDomain:
            self.addProb(elt, eps)
        self.normalize()

    def condition(self, test):
        return DDist({e: self.prob(e) for e in self.support() if test(e)})

    # Used to be called map
    def project(self, f):
        """
        Return a new distribution with the same probabilities, but
        with each element replaced by f(elt)
        """
        newD = {}
        for key, val in self.__d.items():
            newK = f(key)
            newD[newK] = val + newD.get(newK, 0.0)
        return DDist(newD)

    def transitionUpdate(self, tDist):
        new = {}
        for sPrime in self.support():  # for each old state sPrime
            tDistS = tDist(sPrime)  # distribution over ending states
            if tDistS is None:  # terminal state
                continue
            oldP = self.prob(sPrime)
            for s in tDistS.support():
                # prob of transitioning to s from sPrime
                new[s] = new.get(s, 0.0) + tDistS.prob(s) * oldP
        self.__d = new

    def obsUpdate(self, obs_model, obs):
        for si in self.support():
            self.mulProb(si, obs_model(si).prob(obs))
        z = sum(self.prob(e) for e in self.support())
        if 0.0 < z:
            self.normalize(z)
        return z

    def obsUpdates(self, obs_models, observations):
        # Avoids explicitly computing the joint distribution
        assert len(obs_models) == len(observations)
        for si in self.support():
            combo = to_tuple(si)
            for obs_model, obs in zip(obs_models, observations):
                self.mulProb(si, obs_model(*combo).prob(obs))
                combo = combo + (obs,)
        z = sum(self.prob(e) for e in self.support())
        if 0.0 < z:
            self.normalize(z)
        return z

    def __repr__(self):
        return '{}{{{}}}'.format(self.__class__.__name__, ', '.join(
            '{}: {:.5f}'.format(*pair) for pair in sorted(
                self.__d.items(), key=lambda pair: pair[1])))

    def __hash__(self):
        return hash(frozenset(self.__d.items()))

    def __eq__(self, other):
        return self.__d == other.__d

    def __ne__(self, other):
        return self.__d != other.__d


######################################################################
#   Special cases

def DeltaDist(v):
    """
    Distribution with all of its probability mass on value C{v}
    """
    return DDist({v: 1.0})


def UniformDist(elts):
    """
    Uniform distribution over a given finite set of C{elts}
    @param elts: list of any kind of item
    """
    p = 1.0 / len(elts)
    return DDist({e: p for e in elts})


class MixtureDist(DiscreteDist):
    """
    A mixture of two probability distributions, d1 and d2, with
    mixture parameter p.  Probability of an
    element x under this distribution is p * d1(x) + (1 - p) * d2(x).
    It is as if we first flip a probability-p coin to decide which
    distribution to draw from, and then choose from the appropriate
    distribution.

    This implementation is lazy;  it stores the component
    distributions.  Alternatively, we could assume that d1 and d2 are
    DDists and compute a new DDist.
    """

    def __init__(self, d1, d2, p):
        self.d1 = d1
        self.d2 = d2
        self.p = p
        self.binom = DDist({True: p, False: 1 - p})

    def prob(self, elt):
        return self.p * self.d1.prob(elt) + (1 - self.p) * self.d2.prob(elt)

    def draw(self):
        if self.binom.draw():
            return self.d1.draw()
        else:
            return self.d2.draw()

    def support(self):
        return list(set(self.d1.support()) | set(self.d2.support()))

    def makeDDist(self):
        return DDist({e: self.prob(e) for e in self.support()})

    def __str__(self):
        result = 'MixtureDist({'
        elts = self.support()
        for x in elts[:-1]:
            result += str(x) + ' : ' + str(self.prob(x)) + ', '
        result += str(elts[-1]) + ' : ' + str(self.prob(elts[-1])) + '})'
        return result

    __repr__ = __str__


def mixDDists(dists):
    support = {key for dist in dists for key in dist.support()}
    assert support
    return DDist({key: sum(weight*dist.prob(key) for dist, weight in dists.items())
                  for key in support})


def MixtureDD(d1, d2, p):
    """
    A mixture of two probability distributions, d1 and d2, with
    mixture parameter p.  Probability of an
    element x under this distribution is p * d1(x) + (1 - p) * d2(x).
    It is as if we first flip a probability-p coin to decide which
    distribution to draw from, and then choose from the appropriate
    distribution.

    This implementation is eager: it computes a DDist
    distributions.  Alternatively, we could assume that d1 and d2 are
    DDists and compute a new DDist.
    """
    return mixDDists({d1: p, d2: 1 - p})


def triangleDist(peak, halfWidth, lo=None, hi=None):
    """
    Construct and return a DDist over integers. The
    distribution will have its peak at index C{peak} and fall off
    linearly from there, reaching 0 at an index C{halfWidth} on
    either side of C{peak}.  Any probability mass that would be below
    C{lo} or above C{hi} is assigned to C{lo} or C{hi}
    """
    d = {clip(peak, lo, hi): 1}
    total = 1
    fhw = float(halfWidth)
    for offset in range(1, halfWidth):
        p = (halfWidth - offset) / fhw
        incrDictEntry(d, clip(peak + offset, lo, hi), p)
        incrDictEntry(d, clip(peak - offset, lo, hi), p)
        total += 2 * p
    for elt, value in d.items():
        d[elt] = value / total
    return DDist(d)


def squareDist(lo, hi, loLimit=None, hiLimit=None):
    """
    Construct and return a DDist over integers.  The
    distribution will have a uniform distribution on integers from
    lo to hi-1 (inclusive).
    Any probability mass that would be below
    C{lo} or above C{hi} is assigned to C{lo} or C{hi}.
    """
    d = {}
    p = 1.0 / (hi - lo)
    for i in range(lo, hi):
        incrDictEntry(d, clip(i, loLimit, hiLimit), p)
    return DDist(d)


def JDist(PA, *PBgAs, **kwargs):
    """
    Create a joint distribution on P(A, B) (in that order),
    represented as a C{DDist}
        
    @param PA: a C{DDist} on some random var A
    @param PBgA: a conditional probability distribution specifying
    P(B | A) (that is, a function from elements of A to C{DDist}
    on B)
    """
    joint = PA.project(to_tuple)
    for PBgA in PBgAs:
        d = {}
        for a in joint.support():
            PB = PBgA(*a)
            for b in PB.support():
                combo = a + (b,)
                d[combo] = joint.prob(a) * PB.prob(b)
        joint = DDist(d, **kwargs)
    return joint


def JDistIndep(*PAs):
    """
    Create a joint distribution on P(A, B) (in that order),
    represented as a C{DDist}.  Assume independent.
        
    @param PA: a C{DDist} on some random var A
    @param PB: a C{DDist} on some random var B
    """
    # TODO: could reduce to JDist
    joint = {}
    supports = [PA.support() for PA in PAs]
    for combo in product(*supports):
        joint[combo] = np.prod([PA.prob(x) for PA, x in zip(PAs, combo)])
    return DDist(joint)


def bayesEvidence(PA, PBgA, b):
    """
    @param PBgA: conditional distribution over B given A (function
    from values of a to C{DDist} over B)
    @param PA: prior on A
    @param b: evidence value for B = b 
    @returns: P(A | b)
    """
    # Remember that the order of the variables will be A, B
    return JDist(PA, PBgA).conditionOnVar(1, b)


def totalProbability(PA, PBgA):
    PB = JDist(PA, PBgA)
    return PB.project(lambda pair: pair[1])


######################################################################
#   Continuous distribution

class GaussianDistribution(Distribution):
    """
    Basic one-dimensional Gaussian.  
    """

    def __init__(self, gmean=0., variance=None, stdev=None):
        self.mean = gmean
        if variance:
            assert stdev is None
            self.var = variance
            self.stdev = math.sqrt(self.var)
        elif stdev:
            assert variance is None
            self.stdev = stdev
            self.var = stdev ** 2
        else:
            raise Exception('Have to specify variance or stdev')

    def __str__(self):
        return 'Normal(' + prettyString(self.mean) + ', ' + \
               prettyString(self.var) + ')'

    def prob(self, v):
        # TODO: zero prob is v is the wrong type
        return gaussian(v, self.mean, self.stdev)

    def mean(self):
        return self.mean

    def mode(self):
        return self.mean

    def variance(self):
        return self.var

    def cdf(self, value):
        return stats.norm(self.mean, self.stdev).cdf(value)

    def pnm(self, delta):
        return gaussPNM(self.stdev, delta)

    def draw(self):
        return random.normalvariate(self.mean, self.stdev)

    def update(self, obs, variance):
        varianceSum = self.var + variance
        self.mean = ((variance * self.mean) + (self.var * obs)) / varianceSum
        self.var = self.var * variance / varianceSum
        self.stdev = math.sqrt(self.var)

    def move(self, new, variance):
        self.mean = new
        self.var = self.var + variance
        self.stdev = math.sqrt(self.var)

    def reset(self, new, variance):
        self.mean = new
        self.var = variance
        self.stdev = math.sqrt(self.var)


class LogNormalDistribution(Distribution):
    """
    log d ~ Normal(mu, sigma)
    Note that in some references we use \rho = 1/\sigma^2
    """

    def __init__(self, mu, sigma):
        self.mu = mu  # this is log of what we'd think of as mu
        self.sigma = sigma

    def prob(self, v):
        return gaussian(math.log(v), self.mu, self.sigma)

    def mean(self):
        return exp(self.mu + self.sigma ^ 2 / 2)

    def mode(self):
        return exp(self.mu - self.sigma ^ 2)

    def median(self):
        return exp(self.mu)

    def variance(self):
        ssigma = self.sigma ^ 2
        return exp(2 * self.mu + ssigma) * (exp(ssigma) - 1)

    def draw(self):
        return exp(random.normalvariate(self.mu, self.sigma))


def fixSigma(sigma, ridge=1e-20):
    # Can pass in ridge > 0 to ensure minimum eigenvalue is always >= ridge
    good = True
    for i in range(len(sigma)):
        for j in range(i):
            if sigma[i, j] != sigma[j, i]:
                if abs(sigma[i, j] - sigma[j, i]) > 1e-10:
                    print('found asymmetry mag:',
                             abs(sigma[i, j] - sigma[j, i]))
                good = False
    if not good:
        sigma = (sigma + sigma.T) / 2

    eigs = linalg.eigvalsh(sigma)
    if any([type(eig) in (complex, complex128) for eig in eigs]):
        print('Complex eigs', eigs)
        # Real symmetrix matrix should have real eigenvalues!
        raise Exception('Complex eigenvalues in COV')
    minEig = min(eigs)
    if minEig < ridge:
        if minEig < 0:
            print('Not positive definite', minEig)
        print('Added to matrix', (ridge - minEig))
        # if -minEig > 1e-5:
        #     raw_input('okay?')
        sigma = sigma + 2 * (ridge - minEig) * identity(len(sigma))
    return sigma


# Uses numpy matrices
# pose4 is a big hack to deal with a case when the last element is a rotation
class MultivariateGaussianDistribution(Distribution):
    def __init__(self, mu, sigma, pose4=False):
        mu = mat(mu)
        if mu.shape[1] != 1 and mu.shape[0] == 1:
            mu = mu.T
        self.mu = mu  # column vector
        if type(sigma) in (list, tuple):
            sigma = makeDiag(sigma)
        self.sigma = fixSigma(mat(sigma))  # square pos def matrix
        self.pose4 = pose4

    def copy(self):
        return MultivariateGaussianDistribution(
            np.copy(self.mu), np.copy(self.sigma), self.pose4)

    def prob(self, v):
        d = len(v)
        if type(v) in (tuple, list):
            v = np.mat(v).T
        norm = math.sqrt((2 * math.pi) ** d * linalg.det(self.sigma))
        diff = v - self.mu
        if diff.shape == (1, 4):
            diff = diff.T
        if self.pose4:
            diff[3][0] = fixAnglePlusMinusPi(diff[3][0])
        return float(exp(-0.5 * diff.T * self.sigma.I * diff) / norm)

    def logProb(self, v):
        d = len(v)
        norm = math.sqrt((2 * math.pi) ** d * linalg.det(self.sigma))
        diff = v - self.mu
        if diff.shape == (1, 4):
            diff = diff.T
        if self.pose4:
            diff[3][0] = fixAnglePlusMinusPi(diff[3][0])
        return -0.5 * diff.T * self.sigma.I * diff - np.log(norm)

    def discreteProb(self, ranges):
        # No good way to do this!  Maybe eventually make a table or something
        # For now, super wrong, assume diagonal
        ans = 1.0
        variances = tuple(np.diag(self.sigma))
        for (box, var, mu) in zip(ranges, variances, tuple(self.mu)):
            g = scipy.stats.norm(mu, np.sqrt(var))
            assert box[1] > box[0]
            ans *= g.cdf(box[1]) - g.cdf(box[0])
        return ans

    def marginal(self, indices):
        assert not self.pose4
        mmu = self.mu.take(indices).T
        mcov = self.sigma.take(indices, axis=0).take(indices, axis=1)
        return MultivariateGaussianDistribution(mmu, mcov)

    def conditional(self, indices2, values2, indices1, xadd=op.add):
        assert not self.pose4
        # Mean of indices1, conditioned on indices2 = values2
        mu1 = self.mu.take(indices1).T
        mu2 = self.mu.take(indices2).T
        sigma11 = self.sigma.take(indices1, axis=0).take(indices1, axis=1)
        sigma12 = self.sigma.take(indices1, axis=0).take(indices2, axis=1)
        sigma21 = self.sigma.take(indices2, axis=0).take(indices1, axis=1)
        sigma22 = self.sigma.take(indices2, axis=0).take(indices2, axis=1)
        sigma22I = sigma22.I
        mu1g2 = xadd(mu1, sigma12 * sigma22I * xadd(values2, -mu2))
        sigma1g2 = fixSigma(sigma11 - sigma12 * sigma22I * sigma21)
        return MultivariateGaussianDistribution(mu1g2, sigma1g2)

    def difference(self, indices1, indices2, xadd=op.add):
        assert not self.pose4
        # dist of indices1 - indices2
        mu1 = self.mu.take(indices1).T
        mu2 = self.mu.take(indices2).T
        sigma11 = self.sigma.take(indices1, axis=0).take(indices1, axis=1)
        sigma21 = self.sigma.take(indices2, axis=0).take(indices1, axis=1)
        sigma12 = self.sigma.take(indices1, axis=0).take(indices2, axis=1)
        sigma22 = self.sigma.take(indices2, axis=0).take(indices2, axis=1)

        mudiff = xadd(mu1, -mu2)
        sigmadiff = fixSigma(sigma11 + sigma22 - sigma21 - sigma12)
        return MultivariateGaussianDistribution(mudiff, sigmadiff)

    def corners(self, p, xadd=op.add, noZ=False):
        assert not self.pose4
        # Generate points along each major axis
        pts = []
        if noZ:
            smallSigma = self.sigma.take([0, 1, 3], axis=0).take([0, 1, 3], axis=1)
            (eigVals, eigVecs) = linalg.eigh(smallSigma)
            for (val, vec) in zip(eigVals, eigVecs.T):
                # Just do it for X, Y, Th;  no variation in Z 
                off3 = math.sqrt(val) * p * vec.T / linalg.norm(vec)
                offset = mat([off3[0, 0], off3[1, 0], 0, off3[2, 0]]).T
                pts.append(xadd(self.mu, offset))
                pts.append(xadd(self.mu, -offset))
        else:
            (eigVals, eigVecs) = linalg.eigh(self.sigma)
            for (val, vec) in zip(eigVals, eigVecs.T):
                offset = math.sqrt(val) * p * vec.T / linalg.norm(vec)
                pts.append(xadd(self.mu, offset))
                pts.append(xadd(self.mu, -offset))
        return pts

    def mean(self):
        return self.mu

    def meanTuple(self):
        return tuple(self.mu.T.tolist()[0])

    def mode(self):
        return self.mu

    def variance(self):
        return self.sigma

    def modeTuple(self):
        return self.meanTuple()

    def modeVar(self):
        return (self.modeTuple(), self.varTuple())

    # if diagonal
    def varTuple(self):
        return tuple(self.sigma[i, i] for i in range(self.sigma.shape[0]))

    def pnm(self, deltas):
        assert not self.pose4
        # Amount of probability mass within delta of mean. Treating
        # the dimensions independently; deltas is a vector; returns
        # a vector of results
        return [gaussPNM(math.sqrt(self.sigma[i, i]), deltas[i]) \
                for i in range(len(deltas))]

    def pn(self, value, deltas):
        assert not self.pose4
        # Amount of probability mass within delta of value
        # Value is a column vector of same dim as mu; so is delta
        return [gaussPN(value[i], deltas[i], float(self.mu[i]),
                        math.sqrt(self.sigma[i, i])) \
                for i in range(len(deltas))]

    # Special hack for when we know this is a vector of poses
    def pnPoses(self, value, deltas):
        assert not self.pose4
        # Amount of probability mass within delta of value
        # Value is a column vector of same dim as mu; so is delta
        result = []
        for i in range(len(deltas)):
            if mod(i, 4) == 3:
                result.append(gaussPNAngle(value[i], deltas[i],
                                           float(self.mu[i]),
                                           math.sqrt(float(self.sigma[i, i]))))
            else:
                result.append(gaussPN(value[i], deltas[i],
                                      float(self.mu[i]),
                                      math.sqrt(float(self.sigma[i, i]))))
        return result

    def draw(self):
        return np.random.multivariate_normal(self.mu.flat, self.sigma)

    def kalmanTransUpdate(self, u, transSigma):
        if type(u) in (list, tuple):
            u = np.mat(u).T
        mu = self.mu + u
        if self.pose4:
            mu[3, 0] = fixAnglePlusMinusPi(mu[3, 0])
        sigma = self.sigma + transSigma
        return MultivariateGaussianDistribution(mu, sigma)

    def kalmanObsUpdate(self, obs, obsSigma):
        if type(obs) in (list, tuple):
            obs = np.mat(obs).T
        innovation = np.transpose(obs - self.mu)
        if self.pose4:
            innovation[0, 3] = fixAnglePlusMinusPi(innovation[0, 3])
        innovation_covariance = self.sigma + obsSigma
        kalman_gain = self.sigma * np.linalg.inv(innovation_covariance)
        size = self.mu.shape[0]
        mu = self.mu + kalman_gain * innovation.T
        sigma = (np.eye(size) - kalman_gain) * self.sigma
        return MultivariateGaussianDistribution(mu, sigma, self.pose4)

    def __str__(self):
        return 'G(' + prettyString(self.mu) + ',' + prettyString(self.sigma) + ')'

    __repr__ = __str__

    def __hash__(self):
        return str(self).__hash__()

    def __eq__(self, other):
        return str(self) == str(other)

    def __ne__(self, other):
        return str(self) != str(other)

UNIFORM_LABEL = 'u'

# A mixture of Gaussians, with implicit "leftover" probability assigned to
# a uniform
class GMU(Distribution):
    def __init__(self, components, ranges=None):
        # A list of (mvg, p) pairs;  p's sum to <= 1
        # All same dimensionality (d)
        self.components = [list(c) for c in components]
        # A list of d pairs (lo, hi), expressing the range for the uniform
        self.ranges = ranges
        if ranges:
            self.area = prod((hi - lo) for (lo, hi) in ranges)
        self.uniformWeight = 1 - sum(p for (d, p) in components)
        assert (ranges is None) or (self.uniformWeight < 10e-10)
        self.mixtureProbs = DDist(dict([(i, p) for i, (c, p) in \
                                        enumerate(self.components)] + \
                                       [(UNIFORM_LABEL, self.uniformWeight)]))

    def copy(self):
        return GMU([[d.copy(), p] for (d, p) in self.components], self.ranges)

    # probability that the value is within this range
    def discreteProb(self, ranges):
        return sum(p * d.discreteProb(ranges) for (d, p) in self.components)

    def kalmanTransUpdate(self, u, transSigma):
        # Side effects
        for i in range(len(self.components)):
            self.components[i][0] = \
                self.components[i][0].kalmanTransUpdate(u, transSigma)

    def kalmanObsUpdate(self, obs, obsSigma):
        # Side effects
        for i in range(len(self.components)):
            self.components[i][0] = \
                self.components[i][0].kalmanObsUpdate(obs, obsSigma)
        self.decreaseUniformWeight()

    # Decrease the amount assigned to the uniform and renormalize others
    def decreaseUniformWeight(self, factor=2.0):
        weights = [c[1] for c in self.components]
        oldWeightSum = sum(weights)
        self.uniformWeight = min(self.uniformWeight / factor, 0.999)
        rweights = [w * (1 - self.uniformWeight) / oldWeightSum \
                    for w in weights]
        for (c, w) in zip(self.components, rweights):
            c[1] = w

    def kalmanObsUpdateNoSE(self, obs, obsSigma):
        thing = GMU([[d.kalmanObsUpdate(obs, obsSigma), p] \
                     for (d, p) in self.components])
        thing.decreaseUniformWeight()
        return thing

    def prob(self, x):
        return sum(d.prob(x) * p for (d, p) in self.components) + \
               ((self.uniformWeight / self.area) if self.ranges else 0)

    def mode(self):
        # Just return the mean of the most likely component...even though
        # that might not really be the mode
        c = self.mld()
        if c:
            return c.mu
        return None

    # Diagonal variance of the most likely component, as a list
    def varTuple(self):
        return self.mld().varTuple()

    def modeTuple(self):
        return self.mld().modeTuple()

    def modeVar(self):
        return self.mld().modeVar()

    def draw(self):
        c = self.mixtureProbs.draw()
        if c == UNIFORM_LABEL:
            return tuple(random.randint(l, h) for (l, h) in self.ranges)
        return self.components[c][0].draw()

    # Returns (dist, p) pair
    def mlc(self):
        # Most likely mixture component;  returns None if uniform
        if len(self.components) != 0:
            return miscUtil.argmax(self.components, lambda pair: pair[1])
        return None

    # Returns dist only
    def mld(self):
        # Most likely mixture component;  returns None if uniform
        if len(self.components) != 0:
            return miscUtil.argmax(self.components, lambda pair: pair[1])[0]
        return None

    def __str__(self):
        return 'GMU(' + ', '.join([prettyString(c) for c in self.components]) + ')'

    __repr__ = __str__


def fitGaussianToPoses(data):
    # Data is a matrix of vectors (len mod 4 = 0) representing vec of poses
    # Each column is a random variable
    mu = meanPoses(data)
    return MultivariateGaussianDistribution(mu, covPoses(data, mu))


def meanPoses(data):
    # Go by columns
    mu = []
    for i in range(data.shape[1]):
        if mod(i, 4) == 3:
            mu.append(angleMean(data[:, i]))
        else:
            mu.append(data[:, i].mean())
    return mat(mu).T


def angleMean(data):
    d = data.T.tolist()[0]
    n = len(d)
    return math.atan2(sum([math.sin(x) for x in d]) / n,
                      sum([math.cos(x) for x in d]) / n)


# Rows of data are examples; mu is a column vector
def covPoses(data, mu):
    n = len(mu)
    sigma = mat(zeros([n, n]))
    for x in data:
        # x is a row;  need to do subtraction respecting angles
        delta = tangentSpaceAdd(x.T, -mu)
        sigma += delta * delta.T
    return sigma / n


class CUniformDist(Distribution):
    """
    Uniform distribution over a given finite one dimensional range
    """

    def __init__(self, xMin, xMax):
        self.xMin = xMin
        self.xMax = xMax
        self.p = 1.0 / (xMax - xMin)

    def prob(self, v):
        if self.xMin <= v <= self.xMax:
            return self.p
        return 0

    def draw(self):
        return self.xMin + random.random() * (self.xMax - self.xMin)

######################################################################
# Factored distributions

class ProductDistribution(Distribution):
    def __init__(self, ds=[]):
        self.ds = tuple(ds)

    def prob(self, vs):
        return reduce(operator.mul, [d.prob(v) for (v, d) in zip(vs, self.ds)])

    def mean(self):
        return [d.mean() for d in self.ds]

    def mode(self):
        return [d.mode() for d in self.ds]

    def variance(self):
        return [d.variance() for d in self.ds]

    def update(self, obs, variance):
        for (o, v, d) in zip(obs, variance, self.ds):
            d.update(o, v)

    def move(self, obs, variance):
        for (o, v, d) in zip(obs, variance, self.ds):
            d.move(o, v)

    def reset(self, obs, variance):
        for (o, v, d) in zip(obs, variance, self.ds):
            d.reset(o, v)

    def draw(self):
        return tuple(d.draw() for d in self.ds)


class ProductGaussianDistribution(ProductDistribution):
    """
    Product of independent Gaussians
    """
    def __init__(self, means, stdevs):
        super(ProductGaussianDistribution, self).__init__(
            [GaussianDistribution(m, s) for (m, s) in zip(means, stdevs)])
        

class ProductUniformDistribution(ProductDistribution):
    def __init__(self, means, stdevs, n):
        super(ProductUniformDistribution, self).__init__(
            [CUniformDist(m - n * s, m + n * s) for (m, s) in zip(means, stdevs)])

######################################################################
# Multinomial distribution.  k items, probability p, independent, that
# each one will flip.  How many flips?

def binomialDist(n, p):
    """
    Binomial distribution on C{n} items with probability C{p}.
    """
    return DDist({k: binCoeff(n, k) * p ** k for k in range(0, n + 1)})


def binCoeff(n, k):
    """
    n choose k  (the binomial coefficient)
    """
    if k < n / 2.0:
        return binCoeff(n, n - k)
    else:
        return reduce(operator.mul, [j for j in range(k + 1, n + 1)], 1)

######################################################################
#   Utilities

def to_tuple(e):
    if isinstance(e, tuple):
        return e
    return (e,)


def removeElt(items, i):
    """
    non-destructively remove the element at index i from a list;
    returns a copy;  if the result is a list of length 1, just return
    the element  
    """
    result = items[:i] + items[i + 1:]
    if len(result) == 1:
        return result[0]
    else:
        return result


def incrDictEntry(d, k, v):
    """
    If dictionary C{d} has key C{k}, then increment C{d[k]} by C{v}.
    Else set C{d[k] = v}.
    
    @param d: dictionary
    @param k: legal dictionary key (doesn't have to be in C{d})
    @param v: numeric value
    """
    if k in d:
        d[k] += v
    else:
        d[k] = v


######################################################################
# Regression

# If we want the 1-pnm(delta) after an observation with obsSigma to be
# < epsr, then what does the 1-pnm have to be before the update?

def regressGaussianPNM(epsr, obsSigma, delta):
    # based on observation
    pnmr = 1 - epsr
    ei2 = ss.erfinv(pnmr) ** 2
    part2 = (delta ** 2) / (2 * obsSigma ** 2)
    if ei2 < part2:
        return .99999999
    if pnmr > .99999999:
        raw_input("Erfinv argument too big")
    return 1 - ss.erf(math.sqrt(ei2 - part2))


def regressGaussianPNMTransition(epsr, transSigma, delta):
    # return epsr * 0.8
    # erfinv(1 - epsr) = delta / sqrt(2 * resultVar)
    # 2 * resultVar * erfinv(1 - epsr)**2 = delta**2
    # resultVar = delta**2 / (2 * erfinv(1 - epsr)**2)
    # prevVar + transVar = resultVar
    # So, if resultVar < transVar this is impossible

    denom = (2 * ss.erfinv(1 - epsr) ** 2)
    if denom <= 0:
        print("Error in erf calculation, epsr=", epsr, ol=True)
        return 1.0

    resultVar = (delta ** 2) / denom
    prevVar = resultVar - transSigma ** 2
    if prevVar <= 0:
        return None
    return 1 - ss.erf(delta / math.sqrt(2 * prevVar))


# Amount of probability mass within delta of mean, given
def gaussPNM(sigma, delta):
    return ss.erf(delta / (math.sqrt(2) * sigma))


# Amount of probability mass within delta of value, given a Gaussian
def gaussPN(value, delta, mu, sigma):
    rv = stats.norm(mu, sigma)
    return rv.cdf(value + delta) - rv.cdf(value - delta)


def gaussPNAngle(value, delta, mu, sigma):
    limit1 = fixAnglePlusMinusPi(value - mu - delta)
    limit2 = fixAnglePlusMinusPi(value - mu + delta)
    upper = max(limit1, limit2)
    lower = min(limit1, limit2)
    rv = stats.norm(0, sigma)
    return rv.cdf(upper) - rv.cdf(lower)


# Gauss CDF
def Phi(x):
    return 0.5 + ss.erf(x / math.sqrt(2.0)) / 2.0


def probModeMoved(delta, var, obsVar):
    p = 1 - ss.erf(delta * (var + obsVar) / (math.sqrt(2.0 * obsVar) * var))
    return p


# chiSq = (0.71, 1.06, 1.65, 2.20, 3.36, 4.88, 5.99, 7.78, 9.49, 13.28, 18.47)
# pValue = (0.95, 0.90, 0.80, 0.70, 0.50, 0.30, 0.20, 0.10, 0.05, 0.01, 0.001)

pValue = (1.0, 0.995, 0.975, 0.20, 0.10, 0.05, 0.025, 0.02, 0.01, 0.005, 0.002, 0.001)
chiSqTables = {
    1: (0.0, 0.0000393, 0.000982, 1.642, 2.706, 3.841, 5.024, 5.412, 6.635, 7.879, 9.550, 10.828),
    2: (0.0, 0.0100, 0.0506, 3.219, 4.605, 5.991, 7.378, 7.824, 9.210, 10.597, 12.429, 13.816),
    3: (0.0, 0.0717, 0.216, 4.642, 6.251, 7.815, 9.348, 9.837, 11.345, 12.838, 14.796, 16.266),
    4: (0.0, 0.207, 0.484, 5.989, 7.779, 9.488, 11.143, 11.668, 13.277, 14.860, 16.924, 18.467),
    5: (0.0, 0.412, 0.831, 7.289, 9.236, 11.070, 12.833, 13.388, 15.086, 16.750, 18.907, 20.515),
    6: (0.0, 0.676, 1.237, 8.558, 10.645, 12.592, 14.449, 15.033, 16.812, 18.548, 20.791, 22.458)
}


# Given p value find chiSq

def chiSqFromP(p, nDof):
    chiSq = chiSqTables[nDof]
    for i in range(len(pValue)):
        if p >= pValue[i]:
            if i == 0:
                slope = (chiSq[i + 1] - chiSq[i]) / (pValue[i + 1] - pValue[i])
                return (p - pValue[i]) * slope + chiSq[i]
            else:
                slope = (chiSq[i] - chiSq[i - 1]) / (pValue[i] - pValue[i - 1])
                return (p - pValue[i - 1]) * slope + chiSq[i - 1]
    slope = (chiSq[-2] - chiSq[-1]) / (pValue[-2] - pValue[-1])
    return (p - pValue[-1]) * slope + chiSq[-1]


def tangentSpaceAdd(a, b):
    res = a + b
    for i in range(3, len(res), 4):
        res[i, 0] = fixAnglePlusMinusPi(res[i, 0])
    return res


def fixAnglePlusMinusPi(a):
    """
    A is an angle in radians;  return an equivalent angle between plus
    and minus pi
    """
    pi2 = 2.0 * math.pi
    i = 0
    while abs(a) > math.pi:
        if a > math.pi:
            a = a - pi2
        elif a < -math.pi:
            a = a + pi2
        i += 1
        if i > 10: break  # loop found
    return a


def clip(v, vMin, vMax):
    if vMin is None:
        if vMax is None:
            return v
        else:
            return min(v, vMax)
    else:
        if vMax is None:
            return max(v, vMin)
        else:
            return max(min(v, vMax), vMin)


def gaussian(x, mu, sigma):
    return math.exp(-((x - mu) ** 2 / (2 * sigma ** 2))) / (sigma * math.sqrt(2 * math.pi))


### All much too specific to 2D.  Fix.

def confDist(c1, c2):
    return math.sqrt((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2)


def gauss(mean, var):
    if not type(mean) == np.ndarray:
        mean = np.array([mean[0], mean[1]])
    if not type(var) == np.ndarray:
        var = np.array([[var[0], 0.0], [0.0, var[1]]])
    return MultivariateGaussianDistribution(mean, var)


def composeVariance(var1, var2):
    return var1 + var2


def invComposeVariance(addedVar, resultVar):
    # Assumes np 
    return resultVar - addedVar


def invComposeVarianceLists(addedVar, resultVar):
    # Assumes np 
    return makeDiag(resultVar) - makeDiag(addedVar)


def moveVariance(conf1, conf2):
    dist = confDist(conf1, conf2)
    var = math.ceil(dist) * 0.001
    moveVar = makeDiag((var, var))
    return moveVar


# Assume H  is identity (transforms state into obs)
def varBeforeObs(obsVar, varAfterObs, maxV=1.0):
    # S = VB + VO
    # K = VB * S^-1
    # VA = (I - K) VB
    # VA = (I - VB (VB + VO)^-1) * VB

    # VA = VB - VB (VB + VO)^{-1} VB

    # VA*VB^{-1} = I - VB (VB + VO)^{-1}
    # VB^{-1}*VA*VB^{-1} = VB^{-1} - (VB + VO)^{-1}
    # (VB + VO)^{-1} = VB^{-1}*VA*VB^{-1} - VB^{-1} 
    # (VB + VO)^{-1} = VB^{-1}*(VA*VB^{-1} - I)
    # (VB + VO) = (VA*VB^{-1} - I)^{-1} * VB

    # Urgh.  Hard to invert in general.
    # Assume it's diagonal, and therefore separable
    # So, for a single entry, we have
    # VA = 1 / (1/VO + 1 / VB)
    # VB = VA VO / (VO - VA)

    # ov = undiag(obsVar)
    return tuple((x * y / (x - y) if x > y else maxV)
                 for (x, y) in zip(obsVar, varAfterObs))

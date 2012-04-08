#include <math.h>
#include <memory.h>
#include <stdio.h>
#include <string.h>
#include "mex.h"


#define PER_LINE_SIZE 7

double get_alpha(double *pt, double *line, double *equ) {
    mpt = [(line(1) + line(2)) / 2, (line(3) + line(4)) / 2];
    equ2 = null([pt, 1; mpt, 1])';
    alpha = atan2(abs(equ(1) * equ2(2) - equ2(1) * equ(2)), abs(equ(1) * equ2(1) + equ(2) * equ2(2)));
}


void compute_all_vote(double *vote, double *lines, double *sz, int nlines)
{
    int i = 0, j = 0, k = 0;
    double pt[2];
    
    for(i = 0; i < sz[0]; i++) {
        for(j = 0; j < sz[1]; j++) {
            pt[0] = i; 
            pt[1] = j;
            
            for(k = 0; k < nlines; k++) {
                alpha = get_alpha(pt, lines(k, 1:4), equ(k, :));
                l = sqrt((lines(k, 1) - lines(k, 2))^2 + (lines(k, 3) - lines(k, 4))^2);
                vote(x, y) = vote(x, y) + l * exp(-(alpha / (2 * sigma ^ 2)));                
            }   
        }
    }
}

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] )
     
{ 
    if(nrhs==4){
    }
    else{
		printf("Invalid number of inputs %d\n",nrhs);
		return;
	}
    
    double *lines = (double*)mxGetData(prhs[0]);
    double *equ = (double*)mxGetData(prhs[1]);
    double *sz = (double*)mxGetData(prhs[2]);
    double step = (double)mxGetScalar(prhs[3]);
    
    int nlines = mxGetN(prhs[0]); // 7 by N
    if(mxGetM(prhs[0]) != PER_LINE_SIZE)
    {
        printf("Invalid lines format %d\n", mxGetM(prhs[0]));
        return;
    }
    
    plhs[0] = mxCreateDoubleMatrix(sz[0], sz[1], mxREAL);
    double *vote = (double*)mxGetData(plhs[0]);
    
    memset(vote, 0, sizeof(double) * sz[0] * sz[1]);
    
    compute_all_vote(vote, lines, sz, nlines);
    
    
    return;
}
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

struct rec {
	double y, x, flux, truth_y, truth_x;
	uint8_t flags;
} __attribute__((packed));

struct stats { double ss, st, sd; size_t n, nt, nd; };

static struct rec *load(const char *path, size_t *n)
{
	FILE *f = fopen(path, "rb");
	if (!f) { perror(path); exit(1); }
	if (fseek(f, 0, SEEK_END)) exit(1);
	long z = ftell(f); rewind(f);
	if (z < 0 || z % (long)sizeof(struct rec)) {
		fprintf(stderr, "%s: invalid size\n", path); exit(1);
	}
	*n = (size_t)z/sizeof(struct rec);
	struct rec *r = malloc(*n*sizeof *r);
	if (!r || fread(r, sizeof *r, *n, f) != *n) exit(1);
	fclose(f); return r;
}

static int bin(double f)
{
	if (f < 5000) return 0;
	if (f < 10000) return 1;
	if (f < 16000) return 2;
	if (f < 22000) return 3;
	return 4;
}

static void learn(const struct rec *r, size_t n, int axis,
	double bias[5], double var[5])
{
	double s[5]={0}, ss[5]={0}; size_t nn[5]={0};
	for (size_t k=0;k<n;k++) {
		if (!(r[k].flags&2) || (r[k].flags&1) || !(r[k].flags&4)) continue;
		int b=bin(r[k].flux); double z=axis?r[k].x:r[k].y;
		double t=axis?r[k].truth_x:r[k].truth_y, e=z-t;
		s[b]+=e; ss[b]+=e*e; nn[b]++;
	}
	for(int b=0;b<5;b++) {
		bias[b]=nn[b]?s[b]/nn[b]:0;
		var[b]=nn[b]>1?ss[b]/nn[b]-bias[b]*bias[b]:100;
		if(var[b]<1e-4)var[b]=1e-4;
	}
}

static struct stats run(const struct rec *r,size_t n,int axis,
	const double bias[5],const double var[5],double q,double damp,
	double rfull,double rscale)
{
	struct stats s={0}; double p=0,v=0,P00=1,P01=0,P11=1; int have=0;
	for(size_t k=0;k<n;k++) {
		int full=r[k].flags&1, bright=r[k].flags&2, ok=r[k].flags&4;
		double z=axis?r[k].x:r[k].y, truth=axis?r[k].truth_x:r[k].truth_y;
		if(!have) { if(!full) continue; p=z; v=0; have=1; continue; }
		p += v; v *= damp;
		double a=P00+2*P01+P11, b=damp*(P01+P11), c=damp*damp*P11;
		/* White acceleration over one frame. */
		P00=a+0.25*q; P01=b+0.5*q; P11=c+q;
		if(full || bright) {
			double R;
			if(full) R=rfull;
			else {int ib=bin(r[k].flux);z-=bias[ib];R=rscale*var[ib];}
			double K0=P00/(P00+R),K1=P01/(P00+R),e=z-p;
			p+=K0*e;v+=K1*e;
			double o00=P00,o01=P01;
			P00=(1-K0)*o00;P01=(1-K0)*o01;P11=P11-K1*o01;
		}
		if(ok && !full) {
			double e=p-truth; s.ss+=e*e; s.n++;
			if(bright){s.st+=e*e;s.nt++;}else{s.sd+=e*e;s.nd++;}
		}
	}
	return s;
}

static struct stats hold(const struct rec *r,size_t n,int axis)
{
	struct stats s={0};double p=0;int have=0;
	for(size_t k=0;k<n;k++){
		int full=r[k].flags&1, bright=r[k].flags&2, ok=r[k].flags&4;
		double z=axis?r[k].x:r[k].y,t=axis?r[k].truth_x:r[k].truth_y;
		if(have&&ok&&!full){double e=p-t;s.ss+=e*e;s.n++;if(bright){s.st+=e*e;s.nt++;}else{s.sd+=e*e;s.nd++;}}
		if(full){p=z;have=1;}
	}return s;
}

int main(int argc,char **argv)
{
	if(argc!=3){fprintf(stderr,"usage: %s TRAIN VALID\n",argv[0]);return 2;}
	size_t nt,nv;struct rec*t=load(argv[1],&nt),*v=load(argv[2],&nv);
	double Q[]={1e-5,3e-5,1e-4,3e-4,.001,.003,.01,.03,.1,.3,1};
	double D[]={.8,.9,.95,.98,.99,.995,.999,1};
	double RF[]={.01,.03,.1,.3,1}; double RS[]={.5,1,2,4};
	for(int axis=0;axis<2;axis++){
		double bias[5],var[5],best=INFINITY,bp[4]={0};learn(t,nt,axis,bias,var);
		for(size_t iq=0;iq<sizeof Q/sizeof*Q;iq++)for(size_t id=0;id<sizeof D/sizeof*D;id++)
		for(size_t ir=0;ir<sizeof RF/sizeof*RF;ir++)for(size_t is=0;is<sizeof RS/sizeof*RS;is++){
			struct stats z=run(t,nt,axis,bias,var,Q[iq],D[id],RF[ir],RS[is]);
			double rms=sqrt(z.ss/z.n);if(rms<best){best=rms;bp[0]=Q[iq];bp[1]=D[id];bp[2]=RF[ir];bp[3]=RS[is];}
		}
		struct stats z=run(v,nv,axis,bias,var,bp[0],bp[1],bp[2],bp[3]);
		struct stats h=hold(v,nv,axis);
		printf("%c train %.4f q %.5g damp %.4g Rfull %.3g Rscale %.3g\n",axis?'X':'Y',best,bp[0],bp[1],bp[2],bp[3]);
		printf("%c validation kalman %.4f (transition %.4f dark %.4f) hold %.4f improvement %.3fx n %zu\n",
			axis?'X':'Y',sqrt(z.ss/z.n),sqrt(z.st/z.nt),sqrt(z.sd/z.nd),sqrt(h.ss/h.n),sqrt(h.ss/z.ss),z.n);
		printf("%c bias",axis?'X':'Y');for(int b=0;b<5;b++)printf(" %.3f/%.3f",bias[b],sqrt(var[b]));puts("");
	}
	free(t);free(v);return 0;
}

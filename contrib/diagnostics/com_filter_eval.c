#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

struct rec { double y, x; unsigned char f; } __attribute__((packed));

struct score { double sy, sx; size_t n; };

static struct rec *load(const char *path, size_t *n)
{
	FILE *f=fopen(path,"rb"); if(!f){perror(path);exit(1);}
	fseek(f,0,SEEK_END); long z=ftell(f); rewind(f);
	*n=(size_t)z/sizeof(struct rec); struct rec *r=malloc(*n*sizeof *r);
	if(fread(r,sizeof *r,*n,f)!=*n){perror("read");exit(1);} fclose(f); return r;
}

/* Score the causal prediction immediately before each complete measurement
 * following at least one non-complete frame.  The complete measurement is
 * then revealed to update the estimator. */
static struct score eval(const struct rec *r,size_t n,double ac,double ai,
	double bc,double bi,double damp)
{
	struct score s={0}; double p[2]={0},v[2]={0}; int have=0,gap=0;
	for(size_t k=0;k<n;k++){
		int rej=r[k].f&1, inf=r[k].f&2, complete=!rej&&!inf;
		double z[2]={r[k].y,r[k].x};
		if(!have){if(complete){p[0]=z[0];p[1]=z[1];have=1;}continue;}
		v[0]*=damp;v[1]*=damp;p[0]+=v[0];p[1]+=v[1];
		if(complete&&gap){double ey=p[0]-z[0],ex=p[1]-z[1];s.sy+=ey*ey;s.sx+=ex*ex;s.n++;}
		if(complete||(!rej&&inf)){
			double a=complete?ac:ai,b=complete?bc:bi;
			for(int j=0;j<2;j++){double e=z[j]-p[j];p[j]+=a*e;v[j]+=b*e;}
		}
		gap=complete?0:gap+1;
	}
	return s;
}

int main(int argc,char **argv)
{
	if(argc!=3){fprintf(stderr,"usage: %s TRAIN.bin VALID.bin\n",argv[0]);return 2;}
	size_t nt,nv;struct rec*t=load(argv[1],&nt),*q=load(argv[2],&nv);
	double besty=INFINITY,bestx=INFINITY,bpy[5]={0},bpx[5]={0};
	struct score bsy={0},bsx={0};
	double A[]={0.05,0.1,0.2,0.35,0.5,0.75,1.0};
	double B[]={0,0.0001,0.0003,0.001,0.003,0.01};
	double D[]={0,0.5,0.8,0.95,0.99,0.999};
	for(size_t ia=0;ia<7;ia++)for(size_t ii=0;ii<7;ii++)
	for(size_t ib=0;ib<6;ib++)for(size_t ji=0;ji<6;ji++)for(size_t id=0;id<6;id++){
		struct score s=eval(t,nt,A[ia],A[ii],B[ib],B[ji],D[id]);
		double ry=sqrt(s.sy/s.n),rx=sqrt(s.sx/s.n);
		if(ry<besty){besty=ry;bpy[0]=A[ia];bpy[1]=A[ii];bpy[2]=B[ib];bpy[3]=B[ji];bpy[4]=D[id];bsy=s;}
		if(rx<bestx){bestx=rx;bpx[0]=A[ia];bpx[1]=A[ii];bpx[2]=B[ib];bpx[3]=B[ji];bpx[4]=D[id];bsx=s;}
	}
	struct score vy=eval(q,nv,bpy[0],bpy[1],bpy[2],bpy[3],bpy[4]);
	struct score vx=eval(q,nv,bpx[0],bpx[1],bpx[2],bpx[3],bpx[4]);
	struct score hold=eval(q,nv,1,0,0,0,0);
	struct score frag=eval(q,nv,1,1,0,0,0);
	printf("best Y train ac %.4g ai %.4g bc %.4g bi %.4g damp %.4g: %.4f n %zu\n",
		bpy[0],bpy[1],bpy[2],bpy[3],bpy[4],sqrt(bsy.sy/bsy.n),bsy.n);
	printf("best X train ac %.4g ai %.4g bc %.4g bi %.4g damp %.4g: %.4f n %zu\n",
		bpx[0],bpx[1],bpx[2],bpx[3],bpx[4],sqrt(bsx.sx/bsx.n),bsx.n);
	printf("validation hold:          y %.4f x %.4f n %zu\n",sqrt(hold.sy/hold.n),sqrt(hold.sx/hold.n),hold.n);
	printf("validation full fragment: y %.4f x %.4f n %zu\n",sqrt(frag.sy/frag.n),sqrt(frag.sx/frag.n),frag.n);
	printf("validation axis-tuned:     y %.4f x %.4f n %zu\n",
		sqrt(vy.sy/vy.n),sqrt(vx.sx/vx.n),vy.n);
	free(t);free(q);return 0;
}

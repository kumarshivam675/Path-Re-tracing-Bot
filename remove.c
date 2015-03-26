#include<stdio.h>
int main()
{
    FILE *inp,*out;
    inp = fopen("playerc.h","r");
    out = fopen("pla.h","w");
    char line[100];
    char newline[100];
    while(!feof(inp))
    {
        fgets(line,sizeof(line),inp);
        int i = 5;
        while(i<100)
        {
        newline[i-5] = line[i];
        i++;
        }
        fprintf(out,"%s",newline);
    }
    } 

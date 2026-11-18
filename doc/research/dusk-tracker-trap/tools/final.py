import sys; sys.argv=['x', sys.argv[1] if len(sys.argv) > 1 else 'results.txt']
exec(open(__file__.replace('final.py', 'pair.py')).read())
B='d1'
dk=lambda f: (lambda k: k[1]=='wx-dusk' and f(k))
summ('wx-dusk all', pairs(dk(lambda k: True), B=B))
for b in ['fry','fry-hq','flat','flat-hq']:
    summ(f'  {b}', pairs(dk(lambda k,b=b: k[2]==b), B=B))
for g in [('1','1'),('0.983','1'),('0.975','1'),('1','1.02')]:
    summ(f'  Vout {g[0]} Vin {g[1]}', pairs(dk(lambda k,g=g: (k[3],k[4])==g), B=B))
for m in ['clean','gauss','meas']:
    summ(f'  {m}', pairs(dk(lambda k,m=m: k[5]==m), B=B))
print()
for sc in sorted({k[1] for k in rows if k[0]==B and k[1]!='wx-dusk'}):
    summ(sc, pairs(lambda k, sc=sc: k[1]==sc, B=B))

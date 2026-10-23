"""Add procedural parquet geometry; collision floor remains unchanged."""
from pathlib import Path
def apply_style(root):
    for p in root.glob('*fixed.wbt'):
        s=p.read_text().split('# PARQUET VISUALS')[0]
        size=6 if '_A_' in p.name else 3
        parts=['# PARQUET VISUALS\nGroup { children [']
        for i in range(size*2):
            for j in range(size*2):
                for k in range(5):
                    x,y=(i*.5+.05+k*.1,j*.5+.25) if (i+j)%2==0 else (i*.5+.25,j*.5+.05+k*.1)
                    sx,sy=(.097,.497) if (i+j)%2==0 else (.497,.097)
                    shade=.02*((i*7+j*3+k)%5)
                    c=(.49+shade,.29+shade,.13+shade)
                    parts.append(f'Transform {{ translation {x} {y} -0.0095 children [ Shape {{ appearance PBRAppearance {{ baseColor {c[0]} {c[1]} {c[2]} roughness 0.8 metalness 0 }} geometry Box {{ size {sx} {sy} 0.001 }} }} ] }}')
        parts.append('] }\n')
        p.write_text(s+'\n'+'\n'.join(parts))
    connected=root/'component_B_connected_test.wbt'
    connected.write_text((root/'component_B_fixed.wbt').read_text().replace('range 1.0','range 4.0'))
    print('Parquet applied to repaired worlds, including connected diagnostic.')

if __name__=='__main__':apply_style(Path(__file__).resolve().parent/'worlds')

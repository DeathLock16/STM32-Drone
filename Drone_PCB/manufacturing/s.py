import pandas as pd

def convert_kicad_pos_to_jlc(infile, outfile="jlcpcb-pos.csv"):
    """
    Convert KiCad .pos file to JLCPCB placement file format.
    
    Required JLCPCB columns:
        Designator | Mid X | Mid Y | Layer | Rotation
    """

    # Load KiCad position file
    df = pd.read_csv(infile)

    # Build output table in JLCPCB format
    out = pd.DataFrame()
    out["Designator"] = df["Ref"]
    out["Mid X"] = df["PosX"]
    out["Mid Y"] = df["PosY"]
    out["Layer"] = df["Side"].str.capitalize()  # Converts top→Top, bottom→Bottom
    out["Rotation"] = df["Rot"]

    # Save converted file
    out.to_csv(outfile, index=False)
    
    print(f"Converted file saved as: {outfile}")


# Example usage:
convert_kicad_pos_to_jlc("Drone_PCB-all-pos.csv")

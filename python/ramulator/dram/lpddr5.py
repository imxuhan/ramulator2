import math

from ramulator.dram.spec import DRAMStandard, TimingConstraint


class LPDDR5(DRAMStandard):
    name = "LPDDR5"
    internal_prefetch_size = 16      # BL16
    read_latency = "nCL + nBL_min"

    # ---- Hierarchy (level name -> init state) ----
    levels = {
        "Channel":      "N_A",
        "Rank":         "N_A",
        "BankGroup":    "N_A",
        "Bank":         "Closed",
        "Row":          "Closed",
        "Column":       "N_A",
    }

    # ---- Commands ----
    commands = [
        "ACT1", "ACT2", "PREpb", "PREab",
        "CAS_RD", "CAS_WR",
        "RD", "WR", "RDA", "WRA",
        "REFab", "REFpb",
    ]

    # ---- CA bus cycle count per command ----
    # LPDDR5 CA is DDR -> all commands are 1 nCK.
    # ACT-1 and ACT-2 are separate 1 nCK commands (interleaving allowed between them).
    command_cycles = {}

    # ---- States ----
    states = ["Opened", "Closed", "Activating", "N_A"]

    # ---- Timing parameters ----
    timing_params = [
        # nBL_min = BL/n_min(BL16); nBL_max = BL/n_max(BL16)
        # (JESD209-5C Table 339).
        "rate", "nBL_min", "nBL_max", "nCL", "nRCD", "nRP", "nRPab", "nRAS", "nRC",
        "nWR", "nRTP", "nCWL", "nPPD",
        "nCCDS", "nCCDL", "nCCDS_WR", "nCCDL_WR",
        "nRRDS", "nRRDL", "nWTRS", "nWTRL",
        "nFAW", "nRFC", "nRFCpb", "nREFI", "nREFIpb",
        "nWCKPST",
        "nCAS",
        "nAAD",
        "nCS", "tCK_ps",
        "nPBR2PBR", "nPBR2ACT",
    ]

    # ---- External request types ----
    supported_requests = {
        "Read": "RD",
        "Write": "WR",
    }

    # ---- Timing constraints ----
    timing_constraints = [
        # Bus occupancy constraints are auto-generated from command_cycles.
        # Channel — data bus occupancy (JESD209-5C Table 339 BL/n_min).
        TimingConstraint(level="Channel", preceding=["RD", "RDA"], following=["RD", "RDA"], latency="nBL_min"),
        TimingConstraint(level="Channel", preceding=["WR", "WRA"], following=["WR", "WRA"], latency="nBL_min"),

        # Bank — CAS must immediately precede RD/WR (JESD209-5C Tables 361-362;
        # nCAS = 0 means next cycle).
        TimingConstraint(level="Bank", preceding=["CAS_RD"], following=["RD", "RDA"], latency="nCAS"),
        TimingConstraint(level="Bank", preceding=["CAS_WR"], following=["WR", "WRA"], latency="nCAS"),

        # Rank — different-BG column timing (JESD209-5C Table 342).
        TimingConstraint(level="Rank", preceding=["RD", "RDA"], following=["RD", "RDA"], latency="nCCDS"),
        TimingConstraint(level="Rank", preceding=["WR", "WRA"], following=["WR", "WRA"], latency="nCCDS_WR"),
        # Rank — read-to-write turnaround (JESD209-5C Tables 348 and 351).
        TimingConstraint(level="Rank", preceding=["RD", "RDA"], following=["WR", "WRA"], latency="nCL + nBL_min + 2 - nCWL"),
        # Rank — write-to-read turnaround (JESD209-5C Table 342).
        TimingConstraint(level="Rank", preceding=["WR", "WRA"], following=["RD", "RDA"], latency="nCWL + nBL_min + nWTRS"),
        # Rank — sibling rank switching, local bus-clear model.
        TimingConstraint(level="Rank", preceding=["RD", "RDA"], following=["RD", "RDA", "WR", "WRA"], latency="nBL_min + nCS", window=1, sibling=True),
        TimingConstraint(level="Rank", preceding=["WR", "WRA"], following=["RD", "RDA"], latency="nCL + nBL_min + nCS - nCWL", window=1, sibling=True),
        # Rank — column command to PREab (JESD209-5C Table 340).
        TimingConstraint(level="Rank", preceding=["RD"], following=["PREab"], latency="nRTP"),
        TimingConstraint(level="Rank", preceding=["WR"], following=["PREab"], latency="nCWL + nBL_min + 1 + nWR"),
        # Rank — RAS timing (JESD209-5C Tables 237, 340-342, and 381).
        TimingConstraint(level="Rank", preceding=["ACT2"], following=["ACT2", "REFpb"], latency="nRRDS"),
        TimingConstraint(level="Rank", preceding=["ACT1"], following=["ACT1"], latency="nFAW", window=4),
        TimingConstraint(level="Rank", preceding=["ACT2"], following=["PREab"], latency="nRAS"),
        TimingConstraint(level="Rank", preceding=["PREab"], following=["ACT2"], latency="nRPab"),
        # Rank — precharge-to-precharge delay (JESD209-5C Table 381 tPPD).
        TimingConstraint(level="Rank", preceding=["PREpb", "PREab"], following=["PREpb", "PREab"], latency="nPPD"),
        # Rank — refresh entry/recovery (JESD209-5C Tables 237 and 240).
        TimingConstraint(level="Rank", preceding=["ACT2"], following=["REFab"], latency="nRC"),
        TimingConstraint(level="Rank", preceding=["PREpb"], following=["REFab"], latency="nRP"),
        TimingConstraint(level="Rank", preceding=["PREab"], following=["REFab", "REFpb"], latency="nRPab"),
        TimingConstraint(level="Rank", preceding=["RDA"], following=["REFab"], latency="nRP + nRTP"),
        TimingConstraint(level="Rank", preceding=["WRA"], following=["REFab"], latency="nCWL + nBL_min + 1 + nWR + nRP"),
        # tRFCab ends at ACT-2 (JESD209-5C Figure 134, Note 3).
        TimingConstraint(level="Rank", preceding=["REFab"], following=["ACT2"], latency="nRFC"),
        TimingConstraint(level="Rank", preceding=["REFab"], following=["PREab", "REFpb", "REFab"], latency="nRFC"),
        TimingConstraint(level="Rank", preceding=["REFpb"], following=["REFpb"], latency="nPBR2PBR"),
        TimingConstraint(level="Rank", preceding=["REFpb"], following=["REFab"], latency="nRFCpb"),
        # Different-bank recovery likewise ends at ACT-2 (JESD209-5C Tables 237 and 240).
        TimingConstraint(level="Rank", preceding=["REFpb"], following=["ACT2"], latency="nPBR2ACT"),

        # BankGroup — same-BG column timing (JESD209-5C Tables 340-341).
        TimingConstraint(level="BankGroup", preceding=["RD", "RDA"], following=["RD", "RDA"], latency="nCCDL"),
        TimingConstraint(level="BankGroup", preceding=["WR", "WRA"], following=["WR", "WRA"], latency="nCCDL_WR"),
        # Same-BG read-to-write uses BL/n_max (JESD209-5C Tables 347 and 350).
        TimingConstraint(level="BankGroup", preceding=["RD", "RDA"], following=["WR", "WRA"], latency="nCL + nBL_max + 2 - nCWL"),
        # BankGroup — same-group write-to-read (JESD209-5C Tables 339-341:
        # WL + BL/n_max + tWTR_L; nBL_max is the column array cycle time).
        TimingConstraint(level="BankGroup", preceding=["WR", "WRA"], following=["RD", "RDA"], latency="nCWL + nBL_max + nWTRL"),
        # BankGroup — same-BG RAS timing (JESD209-5C Tables 340-341; ACT-2 to ACT-2).
        TimingConstraint(level="BankGroup", preceding=["ACT2"], following=["ACT2"], latency="nRRDL"),

        # Bank — single-bank timing (JESD209-5C Tables 340 and 353; measured from ACT-2).
        TimingConstraint(level="Bank", preceding=["ACT2"], following=["ACT2"], latency="nRC"),
        TimingConstraint(level="Bank", preceding=["ACT2"], following=["RD", "RDA", "WR", "WRA"], latency="nRCD"),
        TimingConstraint(level="Bank", preceding=["ACT2"], following=["PREpb"], latency="nRAS"),
        TimingConstraint(level="Bank", preceding=["PREpb"], following=["ACT2"], latency="nRP"),
        TimingConstraint(level="Bank", preceding=["RD"], following=["PREpb"], latency="nRTP"),
        TimingConstraint(level="Bank", preceding=["WR"], following=["PREpb"], latency="nCWL + nBL_min + 1 + nWR"),
        TimingConstraint(level="Bank", preceding=["RDA"], following=["ACT2"], latency="nRTP + nRP"),
        TimingConstraint(level="Bank", preceding=["WRA"], following=["ACT2"], latency="nCWL + nBL_min + 1 + nWR + nRP"),

        # Bank — per-bank refresh (JESD209-5C Tables 237 and 240; Figure 135).
        TimingConstraint(level="Bank", preceding=["REFpb"], following=["ACT2"], latency="nRFCpb"),
        TimingConstraint(level="Bank", preceding=["ACT2"], following=["REFpb"], latency="nRC"),
        TimingConstraint(level="Bank", preceding=["PREpb"], following=["REFpb"], latency="nRP"),
        TimingConstraint(level="Bank", preceding=["RDA"], following=["REFpb"], latency="nRTP + nRP"),
        TimingConstraint(level="Bank", preceding=["WRA"], following=["REFpb"], latency="nCWL + nBL_min + 1 + nWR + nRP"),
    ]

    # ---- Secondary timing resolution ----
    @classmethod
    def resolve_secondary_timings(cls, timing_dict, org_dict):
        timing_dict["nRRDS"] = cls._resolve_nRRDS(timing_dict["tCK_ps"])
        timing_dict["nRRDL"] = cls._resolve_nRRDL(timing_dict["tCK_ps"])
        timing_dict["nFAW"] = cls._resolve_nFAW(timing_dict["tCK_ps"])
        timing_dict["nRFC"] = cls._resolve_nRFC(org_dict["density"], timing_dict["tCK_ps"])
        timing_dict["nRFCpb"] = cls._resolve_nRFCpb(org_dict["density"], timing_dict["tCK_ps"])
        timing_dict["nREFI"] = cls._resolve_nREFI(timing_dict["tCK_ps"])
        timing_dict["nREFIpb"] = cls._resolve_nREFIpb(timing_dict["tCK_ps"])
        timing_dict["nPBR2PBR"] = cls._resolve_nPBR2PBR(org_dict["density"], timing_dict["tCK_ps"])
        timing_dict["nPBR2ACT"] = cls._resolve_nPBR2ACT(timing_dict["tCK_ps"])

    @staticmethod
    def _resolve_nRRDS(tCK_ps):
        return max(4, math.ceil(5_000 / tCK_ps))

    @staticmethod
    def _resolve_nRRDL(tCK_ps):
        return max(4, math.ceil(5_000 / tCK_ps))

    @staticmethod
    def _resolve_nFAW(tCK_ps):
        # JESD209-5C Table 381: LPDDR5 BG mode tFAW = 20 ns.
        return math.ceil(20_000 / tCK_ps)

    @staticmethod
    def _resolve_nRFC(density, tCK_ps):
        # JESD209-5C Table 240: tRFCab, Enhanced DVFSC disabled.
        if density <= 2048:    tRFC_ns = 130
        elif density <= 4096:  tRFC_ns = 180
        elif density <= 8192:  tRFC_ns = 210
        elif density <= 16384: tRFC_ns = 280
        else:                  tRFC_ns = 380
        return math.ceil(tRFC_ns * 1000 / tCK_ps)

    @staticmethod
    def _resolve_nRFCpb(density, tCK_ps):
        # JESD209-5C Table 240: tRFCpb, Enhanced DVFSC disabled.
        if density <= 2048:    tRFC_ns = 60
        elif density <= 4096:  tRFC_ns = 90
        elif density <= 8192:  tRFC_ns = 120
        elif density <= 16384: tRFC_ns = 140
        else:                  tRFC_ns = 190
        return math.ceil(tRFC_ns * 1000 / tCK_ps)

    @staticmethod
    def _resolve_nREFI(tCK_ps):
        # JESD209-5C Table 240: tREFI is a maximum average interval.
        return 3_906_000 // tCK_ps

    @staticmethod
    def _resolve_nREFIpb(tCK_ps):
        # JESD209-5C Table 240: tREFIpb is a maximum average interval.
        return 488_000 // tCK_ps

    @staticmethod
    def _resolve_nPBR2PBR(density, tCK_ps):
        # JESD209-5C Table 240: tpbR2pbR.
        if density <= 2048: t_ns = 60
        else:               t_ns = 90
        return math.ceil(t_ns * 1000 / tCK_ps)

    @staticmethod
    def _resolve_nPBR2ACT(tCK_ps):
        # JESD209-5C Table 240: tpbr2act for BG mode.
        return math.ceil(7_500 / tCK_ps)


# ---- LPDDR5 JESD209-5C preset data ----

LPDDR5.org_presets = {
    "LPDDR5_8Gb_x16":  {"density": 8192,  "dq": 16, "channel_width": 16, "rank": 1, "bankgroup": 4, "bank": 4, "row": 1<<15, "column": 1<<10},
    "LPDDR5_16Gb_x16": {"density": 16384, "dq": 16, "channel_width": 16, "rank": 1, "bankgroup": 4, "bank": 4, "row": 1<<16, "column": 1<<10},
}

# Primary timings only — secondary timings resolved by resolve_secondary_timings().
# All values in CK cycles (CKR 4:1).
LPDDR5.timing_presets = {
    # JESD209-5C Tables 225 and 340: nRTP = BL/n_min (2) + nRBTP (4).
    # LPDDR5-5500: x16 BG mode, CKR 4:1, RL Set 0, WL Set A
    "LPDDR5_5500": {
        "rate": 5500, "nBL_min": 2, "nBL_max": 4, "nCL": 15, "nRCD": 13, "nRP": 13, "nRPab": 15,
        "nRAS": 29, "nRC": 42, "nWR": 24, "nRTP": 6, "nCWL": 8, "nPPD": 2,
        "nCCDS": 2, "nCCDL": 4, "nCCDS_WR": 2, "nCCDL_WR": 4,
        "nWTRS": 5, "nWTRL": 9, "nWCKPST": 1, "nCAS": 0,
        "nAAD": 8, "nCS": 2, "tCK_ps": 1453,
    },
    # LPDDR5-6400 (tCK = 1250 ps, CK = 800 MHz)
    "LPDDR5_6400": {
        # JESD209-5C Table 339: BG mode, CKR 4:1, WCK > 1600 MHz:
        # nBL_min = BL/n_min(BL16) = 2; nBL_max = BL/n_max(BL16) = 4.
        "rate": 6400, "nBL_min": 2, "nBL_max": 4, "nCL": 17, "nRCD": 15, "nRP": 15, "nRPab": 17,
        "nRAS": 34, "nRC": 49, "nWR": 28, "nRTP": 6, "nCWL": 9, "nPPD": 2,
        "nCCDS": 2, "nCCDL": 4, "nCCDS_WR": 2, "nCCDL_WR": 4,
        "nWTRS": 5, "nWTRL": 10, "nWCKPST": 1, "nCAS": 0,
        "nAAD": 8, "nCS": 2, "tCK_ps": 1250,
    },
}

"""
PINN observer training pipeline.

Trains a Physics-Informed Neural Network (PINN) observer using
telemetry data, digital-twin predictions, and conservation-based
physics loss terms.
"""
import torch
from torch.utils.data import DataLoader
from pathlib import Path
from pinn.model import PhysicsInformedResidualObserver, PhysicsInformedLoss
from pinn.dataset import EngineRunDataset
from typing import List
from core.telemetry_schema.schemas import NormalizedEngineTelemetry, TwinPhysicsPrediction

def train_pinn(telemetry_records: List[NormalizedEngineTelemetry], twin_predictions: List[TwinPhysicsPrediction], epochs: int = 25, lr: float = 0.003) -> PhysicsInformedResidualObserver:
    dataset = EngineRunDataset(telemetry_records, twin_predictions)
    loader = DataLoader(dataset, batch_size=32, shuffle=True)

    model = PhysicsInformedResidualObserver(input_dim=16, hidden_dim=64, output_dim=6)
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    criterion = PhysicsInformedLoss(lambda_mass=0.05, lambda_energy=0.05, lambda_mono=0.02)

    model.train()
    for epoch in range(epochs):
        epoch_loss = 0.0
        for feats, targets, raws in loader:
            optimizer.zero_grad()
            corr, anom, eff = model(feats)
            loss, l_data, l_mass, l_energy, l_mono = criterion(corr, anom, eff, targets, raws)
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item()

    model.eval()
    return model

if __name__ == "__main__":
    print("PINN training script module loaded.")
